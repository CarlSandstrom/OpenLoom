#include "Meshing/Core/3D/RCDT/RCDTMesher.h"

#include "Common/Exceptions/MeshException.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Meshing/Core/3D/General/BoundaryDiscretizer3D.h"
#include "Meshing/Core/3D/General/DiscretizationResult3D.h"
#include "Meshing/Core/3D/General/MeshDebugUtils3D.h"
#include "Meshing/Core/3D/General/MeshOperations3D.h"
#include "Meshing/Core/3D/General/MeshingContext3D.h"
#include "Meshing/Core/3D/General/SizingField3D.h"
#include "Meshing/Core/3D/RCDT/AmbientTetrahedronRemover.h"
#include "Meshing/Core/3D/RCDT/CurveProtectionSubdivider.h"
#include "Meshing/Core/3D/RCDT/CurveSegmentOperations.h"
#include "Meshing/Core/3D/RCDT/MinimumEdgeLengthEstimator.h"
#include "Meshing/Core/3D/RCDT/RCDTMeshExtractor.h"
#include "Meshing/Core/3D/RCDT/RCDTRefiner.h"
#include "Meshing/Core/3D/RCDT/RCDTTetQualityController.h"
#include "Meshing/Core/3D/RCDT/RestrictedTriangulation.h"
#include "Meshing/Core/3D/RCDT/SurfaceMeshSmoother.h"
#include "Meshing/Core/3D/Volume/Delaunay3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Meshing/Data/Base/MeshConnectivity.h"
#include "Meshing/Data/CurveSegmentManager.h"
#include "spdlog/spdlog.h"

#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace Meshing
{

namespace
{

// Resolved before CurveProtectionSubdivider/Delaunay3D run, since the
// subdivider needs a size floor to subdivide against.
double resolveMinimumEdgeLength(const SurfaceMesh3DQualitySettings& qualitySettings,
                                const SizingField3D* sizingField,
                                const std::vector<Point3D>& points)
{
    if (qualitySettings.minimumEdgeLength)
        return *qualitySettings.minimumEdgeLength;
    if (sizingField)
        return MinimumEdgeLengthEstimator::fromSizingField(*sizingField);
    return MinimumEdgeLengthEstimator::fromPointSpacing(points);
}

// Boissonnat-Oudot protecting balls (OPE-176): every curve/corner sample
// point is inserted into the initial triangulation as a WEIGHTED point
// (see RegularPredicates3D) rather than an ordinary one, which forces
// every crease to appear as an exact edge chain in the resulting
// regular triangulation -- this is what lets
// RestrictedTriangulation::classifyFace() disambiguate a
// crease-straddling face reliably instead of guessing. subdivide()
// both sizes those weights (CurveProtectionScheme) and, where a corner's
// radius and a curve's own sampling density are too far apart for a
// single pair of points to bridge, inserts additional curve points to
// close the gap gradually -- see CurveProtectionScheme/
// CurveProtectionSubdivider's own docs for the two properties every
// radius satisfies and how a conflict between them is resolved.
//
// The curve segments are populated here, before RestrictedTriangulation::buildFrom()
// runs, not after: classifyFace() consults the CurveSegmentManager to
// recognize a genuinely protected edge (see its doc), so that lookup needs
// the curve network in place for the very first classification pass, not
// just for ones triggered later by refinement.
void seedAmbientTriangulation(MeshingContext3D& context,
                              DiscretizationResult3D& discretizationResult,
                              const Geometry3D::GeometryCollection3D& geometry,
                              const Topology3D::Topology3D& topology,
                              double minimumEdgeLength)
{
    const auto pointWeights =
        CurveProtectionSubdivider::subdivide(discretizationResult, topology, geometry, minimumEdgeLength);

    const auto& meshData = context.getMeshData();
    const auto delaunayResult = Delaunay3D::triangulate(context.getOperations(),
                                                        discretizationResult.points,
                                                        discretizationResult.geometryIds,
                                                        pointWeights);

    spdlog::info("RCDTMesher::buildInitial: Delaunay3D produced {} nodes, {} elements",
                 meshData.getNodeCount(), meshData.getElementCount());

    context.getMutator().setCurveSegmentManager(
        CurveSegmentOperations::buildCurveSegments(topology, geometry,
                                                   discretizationResult.edgeIdToPointIndicesMap,
                                                   delaunayResult.pointIndexToNodeIdMap,
                                                   discretizationResult.edgeParameters));

    spdlog::info("RCDTMesher::buildInitial: {} curve segments added",
                 meshData.getCurveSegmentManager().size());
}

void logDefectiveFaceRemoval(const DefectiveFaceRemovalSummary& defectRemoval)
{
    if (defectRemoval.chordFacesRemoved > 0)
        spdlog::info("RCDTMesher::runPipeline: removed {} same-curve chord faces", defectRemoval.chordFacesRemoved);
    if (defectRemoval.excessFacesRemoved > 0)
        spdlog::info("RCDTMesher::runPipeline: removed {} excess restricted faces", defectRemoval.excessFacesRemoved);

    const size_t remainingDefects = defectRemoval.remainingMissingFaceEdges + defectRemoval.remainingExcessFaceEdges +
                                    defectRemoval.remainingSurfaceMismatchEdges;
    if (remainingDefects > 0)
    {
        spdlog::info("RCDTMesher::runPipeline: {} non-manifold edges remain — {} holes, {} excess, {} surface mismatch",
                     remainingDefects, defectRemoval.remainingMissingFaceEdges, defectRemoval.remainingExcessFaceEdges,
                     defectRemoval.remainingSurfaceMismatchEdges);
    }
}

// Smoothing only moves the SurfaceMesh3D copy. The same node IDs are still
// referenced by the tetrahedra in the context's live MeshData3D
// (extractVolumeMesh() reads those directly) — sync the smoothed positions
// back so both stay geometrically consistent, rather than only the returned
// copy.
void syncNodePositions(MeshingContext3D& context, const SurfaceMesh3D& surfaceMesh)
{
    auto& mutator = context.getMutator();
    for (size_t nodeId = 0; nodeId < surfaceMesh.nodes.size(); ++nodeId)
    {
        if (context.getMeshData().getNode(nodeId))
            mutator.moveNode(nodeId, surfaceMesh.nodes[nodeId]);
    }
}

// AmbientTetrahedronRemover's flood fill crosses every face that is not
// restricted, so a hole in the restricted boundary lets it walk into the solid
// and delete tetrahedra that belong to the model -- OPE-185's empty BoxWithHole
// mesh. A surface mesh with a hole is still a usable result that reports its
// own defects; a volume mesh missing part of its interior is not.
void requireClosedBoundary(const DefectiveFaceRemovalSummary& defectRemoval)
{
    if (defectRemoval.remainingMissingFaceEdges == 0)
        return;

    OPENLOOM_THROW_MESH(GENERATION_FAILED,
                        "RCDTMesher::meshVolume: the restricted boundary still has holes (" +
                            std::to_string(defectRemoval.remainingMissingFaceEdges) +
                            " edges missing a face), so the solid's interior cannot be separated "
                            "from the ambient tetrahedra");
}

} // namespace

RCDTMesher::RCDTMesher(const Geometry3D::GeometryCollection3D& geometry,
                       const Topology3D::Topology3D& topology,
                       Geometry3D::DiscretizationSettings3D discretizationSettings,
                       SurfaceMesh3DQualitySettings qualitySettings,
                       std::optional<SizingFieldSettings3D> sizingFieldSettings) :
    geometry_(&geometry),
    topology_(&topology),
    discretizationSettings_(discretizationSettings),
    qualitySettings_(qualitySettings),
    sizingFieldSettings_(std::move(sizingFieldSettings))
{
    // A non-positive floor is not "no floor": RestrictedTriangulation sizes its
    // tessellation oracle by it, and SurfaceTessellation builds no cells at all
    // for a target size <= 0, leaving every surface's crossing test with
    // nothing to test against.
    const auto& minimumEdgeLength = qualitySettings_.minimumEdgeLength;
    if (minimumEdgeLength && (!(*minimumEdgeLength > 0.0) || !std::isfinite(*minimumEdgeLength)))
    {
        OPENLOOM_THROW_MESH(INVALID_OPERATION,
                            "RCDTMesher: minimumEdgeLength must be finite and strictly positive when set; "
                            "leave it unset to derive it from the geometry");
    }
}

SurfaceMesh3D RCDTMesher::runPipeline(MeshingContext3D& context,
                                      RestrictedTriangulation& restrictedTriangulation,
                                      bool meshingVolume) const
{
    const double minimumEdgeLength = buildInitial(context, restrictedTriangulation);
    exportMesh3D(context.getMeshData(), "rcdt_initial", 0);

    refine(context, restrictedTriangulation, minimumEdgeLength, meshingVolume);
    exportMesh3D(context.getMeshData(), "rcdt_refined", 1);

    const auto defectRemoval = restrictedTriangulation.removeDefectiveFaces(context.getMeshData());
    logDefectiveFaceRemoval(defectRemoval);
    if (meshingVolume)
        requireClosedBoundary(defectRemoval);

    // Strips every ambient tetrahedron -- both the seed triangulation's
    // outer shell and, for domains with holes, the tetrahedra RCDT kept
    // triangulating interior voids with. getOperations()'s mutator, not
    // getMutator(): the latter validates node removal against a
    // MeshConnectivity snapshot that's only refreshed by an explicit
    // rebuildConnectivity() call, and refine()'s many insertions never call
    // it -- that snapshot is stale by the time we get here. The operations
    // mutator performs no such (now-stale) validation.
    AmbientTetrahedronRemover::remove(context.getMeshData(), context.getOperations().getMutator(),
                                      restrictedTriangulation);

    SurfaceMesh3D surfaceMesh =
        RCDTMeshExtractor::extractSurfaceMesh(context.getMeshData(), restrictedTriangulation, *topology_);

    if (qualitySettings_.smoothingIterations > 0)
    {
        spdlog::info("RCDTMesher: smoothing surface mesh ({} iterations)",
                     qualitySettings_.smoothingIterations);
        // In volume mode the surface nodes are shared with the solid's
        // tetrahedra, which smoothing must not turn inside out.
        std::vector<std::array<size_t, 4>> tetrahedra;
        if (meshingVolume)
            tetrahedra = RCDTMeshExtractor::extractTetrahedra(context.getMeshData());
        SurfaceMeshSmoother::smooth(*geometry_, surfaceMesh, qualitySettings_.smoothingIterations, tetrahedra);
        syncNodePositions(context, surfaceMesh);
    }

    exportMesh3D(context.getMeshData(), "rcdt_smoothed", 2);

    return surfaceMesh;
}

SurfaceMesh3D RCDTMesher::meshSurface()
{
    MeshingContext3D context(*geometry_, *topology_);
    RestrictedTriangulation restrictedTriangulation;
    SurfaceMesh3D surfaceMesh = runPipeline(context, restrictedTriangulation, false);

    // Triangle-only export of the actual output — unlike the exports in
    // runPipeline(), this contains none of the ambient tetrahedralization's
    // interior faces (see RestrictedTriangulation: a triangle whose corners
    // all lie on a CAD surface is not necessarily one of the faces RCDT
    // selected as the boundary there).
    exportSurfaceMesh3D(surfaceMesh, "rcdt_surface_mesh.vtu");

    return surfaceMesh;
}

VolumeMesh3D RCDTMesher::meshVolume()
{
    MeshingContext3D context(*geometry_, *topology_);
    RestrictedTriangulation restrictedTriangulation;

    // The returned SurfaceMesh3D is only needed for the smoother's triangle
    // adjacency inside runPipeline() — smoothing already synced the resulting
    // positions back into the live mesh, so extractVolumeMesh() (reading that
    // live mesh directly) sees the same, consistent positions.
    runPipeline(context, restrictedTriangulation, true);

    return RCDTMeshExtractor::extractVolumeMesh(context.getMeshData(), restrictedTriangulation, *topology_);
}

double RCDTMesher::buildInitial(MeshingContext3D& context, RestrictedTriangulation& restrictedTriangulation) const
{
    spdlog::info("RCDTMesher::buildInitial: discretizing boundary ({} surface samples/direction)",
                 discretizationSettings_.getNumSamplesPerSurfaceDirection());

    // Built before discretization and kept, so the size floor below reads the
    // same h(x) the discretization did -- the point of OPE-181.
    std::optional<SizingField3D> sizingField;
    if (sizingFieldSettings_.has_value())
        sizingField = SizingFieldBuilder3D::build(*geometry_, *topology_, sizingFieldSettings_.value());

    auto discretizationResult =
        BoundaryDiscretizer3D::discretize(*geometry_,
                                          *topology_,
                                          discretizationSettings_,
                                          sizingField ? &sizingField.value() : nullptr);

    spdlog::info("RCDTMesher::buildInitial: {} points after discretization",
                 discretizationResult->points.size());

    const double minimumEdgeLength =
        resolveMinimumEdgeLength(qualitySettings_, sizingField ? &sizingField.value() : nullptr,
                                 discretizationResult->points);
    spdlog::info("RCDTMesher::buildInitial: minimum edge length = {}", minimumEdgeLength);

    seedAmbientTriangulation(context, *discretizationResult, *geometry_, *topology_, minimumEdgeLength);

    const auto& meshData = context.getMeshData();
    const MeshConnectivity connectivity(meshData);
    restrictedTriangulation.buildFrom(meshData, connectivity, *geometry_, *topology_, minimumEdgeLength,
                                      qualitySettings_);

    spdlog::info("RCDTMesher::buildInitial: {} restricted faces",
                 restrictedTriangulation.getRestrictedFaces().size());

    return minimumEdgeLength;
}

void RCDTMesher::refine(MeshingContext3D& context,
                        RestrictedTriangulation& restrictedTriangulation,
                        double minimumEdgeLength,
                        bool includeTetQualityRefinement) const
{
    spdlog::info("RCDTMesher::refine: starting RCDT refinement (tet quality: {})",
                 includeTetQualityRefinement);

    std::unique_ptr<RCDTTetQualityController> tetQualityController;
    if (includeTetQualityRefinement)
    {
        tetQualityController =
            std::make_unique<RCDTTetQualityController>(context.getMeshData(), qualitySettings_);
    }

    RCDTRefiner refiner(context,
                        restrictedTriangulation,
                        qualitySettings_,
                        minimumEdgeLength,
                        tetQualityController.get());
    refiner.refine();
    spdlog::info("RCDTMesher::refine: done");
}

} // namespace Meshing
