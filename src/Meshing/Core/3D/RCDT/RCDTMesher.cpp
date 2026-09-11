#include "Meshing/Core/3D/RCDT/RCDTMesher.h"

#include "Common/Exceptions/MeshException.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Meshing/Core/3D/General/BoundaryDiscretizer3D.h"
#include "Meshing/Core/3D/General/DiscretizationResult3D.h"
#include "Meshing/Core/3D/General/MeshDebugUtils3D.h"
#include "Meshing/Core/3D/General/MeshOperations3D.h"
#include "Meshing/Core/3D/General/MeshingContext3D.h"
#include "Meshing/Core/3D/General/SizingField3D.h"
#include "Meshing/Core/3D/RCDT/AmbientTetrahedronClassifier.h"
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
#include "Meshing/Data/3D/TetrahedralElement.h"
#include "Meshing/Data/Base/MeshConnectivity.h"
#include "Meshing/Data/CurveSegmentManager.h"
#include "spdlog/spdlog.h"

#include <array>

namespace Meshing
{

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
}

SurfaceMesh3D RCDTMesher::runPipeline(MeshingContext3D& context,
                                      RestrictedTriangulation& restrictedTriangulation,
                                      bool includeTetQualityRefinement) const
{
    size_t counter = 0;
    const double minimumEdgeLength = buildInitial(context, restrictedTriangulation);
    Meshing::exportMesh3D(context.getMeshData(), "rcdt_initial", counter);
    ++counter;

    refine(context, restrictedTriangulation, minimumEdgeLength, includeTetQualityRefinement);
    Meshing::exportMesh3D(context.getMeshData(), "rcdt_refined", counter);
    ++counter;

    const auto defectRemoval = restrictedTriangulation.removeDefectiveFaces(context.getMeshData());
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

    removeBoundingTetrahedron(context, restrictedTriangulation);

    SurfaceMesh3D surfaceMesh =
        RCDTMeshExtractor::extractSurfaceMesh(context.getMeshData(), restrictedTriangulation, *topology_);

    if (qualitySettings_.smoothingIterations > 0)
    {
        spdlog::info("RCDTMesher: smoothing surface mesh ({} iterations)",
                     qualitySettings_.smoothingIterations);
        SurfaceMeshSmoother::smooth(*geometry_, surfaceMesh, qualitySettings_.smoothingIterations);

        // Smoothing only moves the SurfaceMesh3D copy above. The same node IDs
        // are still referenced by the ambient tetrahedra in the context's
        // live MeshData3D (extractVolumeMesh() reads those directly) — sync the
        // smoothed positions back so both stay geometrically consistent,
        // rather than only the returned copy.
        auto& mutator = context.getMutator();
        for (size_t nodeId = 0; nodeId < surfaceMesh.nodes.size(); ++nodeId)
        {
            if (context.getMeshData().getNode(nodeId))
                mutator.moveNode(nodeId, surfaceMesh.nodes[nodeId]);
        }
    }

    Meshing::exportMesh3D(context.getMeshData(), "rcdt_smoothed", counter);
    ++counter;

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
    Meshing::exportSurfaceMesh3D(surfaceMesh, "rcdt_surface_mesh.vtu");

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

    // Resolved here, before CurveProtectionSubdivider/Delaunay3D run, since
    // the subdivider needs a size floor to subdivide against.
    double minimumEdgeLength = 0.0;
    if (qualitySettings_.minimumEdgeLength)
        minimumEdgeLength = *qualitySettings_.minimumEdgeLength;
    else if (sizingField)
        minimumEdgeLength = MinimumEdgeLengthEstimator::fromSizingField(*sizingField);
    else
        minimumEdgeLength = MinimumEdgeLengthEstimator::fromPointSpacing(discretizationResult->points);
    spdlog::info("RCDTMesher::buildInitial: minimum edge length = {}", minimumEdgeLength);

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
    const auto pointWeights = CurveProtectionSubdivider::subdivide(
        *discretizationResult, *topology_, *geometry_, minimumEdgeLength);

    auto& meshData = context.getMeshData();

    const auto delaunayResult = Delaunay3D::triangulate(context.getOperations(),
                                                        discretizationResult->points,
                                                        discretizationResult->geometryIds,
                                                        pointWeights);
    const auto& pointIndexToNodeIdMap = delaunayResult.pointIndexToNodeIdMap;

    spdlog::info("RCDTMesher::buildInitial: Delaunay3D produced {} nodes, {} elements",
                 meshData.getNodeCount(), meshData.getElementCount());

    // Populated before RestrictedTriangulation::buildFrom() below, not after:
    // classifyFace() consults the CurveSegmentManager to recognize a
    // genuinely protected edge (see its doc), so that lookup needs the curve
    // network in place for the very first classification pass, not just for
    // ones triggered later by refinement.
    context.getMutator().setCurveSegmentManager(
        CurveSegmentOperations::buildCurveSegments(*topology_, *geometry_,
                                                   discretizationResult->edgeIdToPointIndicesMap,
                                                   pointIndexToNodeIdMap,
                                                   discretizationResult->edgeParameters));

    spdlog::info("RCDTMesher::buildInitial: {} curve segments added",
                 meshData.getCurveSegmentManager().size());

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

void RCDTMesher::removeBoundingTetrahedron(MeshingContext3D& context,
                                           const RestrictedTriangulation& restrictedTriangulation) const
{
    // Strips every ambient tetrahedron -- both the seed triangulation's
    // outer shell (touching the supertet's corners) and, for domains with
    // holes, the tets RCDT kept triangulating interior voids with -- not
    // just the ones literally touching a bounding node. See
    // AmbientTetrahedronClassifier's class docs for why one flood fill
    // handles both.
    const auto ambientTetIdSet =
        AmbientTetrahedronClassifier::classify(context.getMeshData(), restrictedTriangulation);

    // getOperations()'s mutator, not getMutator(): the latter validates node
    // removal against a MeshConnectivity snapshot that's only refreshed by an
    // explicit rebuildConnectivity() call, and refine()'s many insertions
    // never call it -- that snapshot is stale by the time we get here. The
    // operations mutator performs no such (now-stale) validation.
    auto& mutator = context.getOperations().getMutator();
    const auto& meshData = context.getMeshData();
    if (!meshData.getBoundingNodeIds())
        OPENLOOM_THROW_MESH(INVALID_OPERATION, "RCDTMesher::removeBoundingTetrahedron: no bounding tetrahedron in the mesh");
    const std::array<size_t, 4> boundingNodeIds = *meshData.getBoundingNodeIds();

    std::vector<size_t> ambientTetIds;
    for (const auto& [elementId, element] : meshData.getElements())
    {
        if (dynamic_cast<const TetrahedralElement*>(element.get()) && ambientTetIdSet.contains(elementId))
            ambientTetIds.push_back(elementId);
    }

    for (const size_t tetId : ambientTetIds)
        mutator.removeElement(tetId);

    for (const size_t nodeId : boundingNodeIds)
        mutator.removeNode(nodeId);
    mutator.clearBoundingNodeIds();

    spdlog::info("RCDTMesher::removeBoundingTetrahedron: Removed {} ambient tetrahedra "
                 "(true exterior + holes) and 4 bounding nodes",
                 ambientTetIds.size());
}

} // namespace Meshing
