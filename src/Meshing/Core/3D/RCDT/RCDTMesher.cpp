#include "Meshing/Core/3D/RCDT/RCDTMesher.h"

#include "Common/Exceptions/MeshException.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Meshing/Core/3D/General/BoundaryDiscretizer3D.h"
#include "Meshing/Core/3D/General/MeshDebugUtils3D.h"
#include "Meshing/Core/3D/General/MeshOperations3D.h"
#include "Meshing/Core/3D/General/MeshingContext3D.h"
#include "Meshing/Core/3D/General/SizingField3D.h"
#include "Meshing/Core/3D/RCDT/AmbientTetrahedronRemover.h"
#include "Meshing/Core/3D/RCDT/CurveSegmentBuilder.h"
#include "Meshing/Core/3D/RCDT/MinimumEdgeLengthEstimator.h"
#include "Meshing/Core/3D/RCDT/ProtectingBallPlacer.h"
#include "Meshing/Core/3D/RCDT/RCDTMeshExtractor.h"
#include "Meshing/Core/3D/RCDT/RestrictedFaceAudit.h"
#include "Meshing/Core/3D/RCDT/SurfaceDelaunayRefiner.h"
#include "Meshing/Core/3D/RCDT/SurfaceMeshSmoother.h"
#include "Meshing/Core/3D/RCDT/TetrahedronDelaunayRefiner.h"
#include "Meshing/Core/3D/Volume/Delaunay3D.h"
#include "Meshing/Data/3D/DiscretizationResult3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Meshing/Data/CurveSegmentManager.h"
#include "spdlog/spdlog.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

namespace Meshing
{

namespace
{

// The size floor: SurfaceTessellation's cell size in WeightedDualRestriction.
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

// Boissonnat-Oudot protecting balls: every corner and curve point is
// inserted into the initial triangulation as a weighted point (see
// RegularPredicates3D), with the radius ProtectingBallPlacer gives it, so that
// every crease appears as an edge chain of the regular triangulation.
void seedAmbientTriangulation(MeshingContext3D& context,
                              DiscretizationResult3D& discretizationResult,
                              const Geometry3D::GeometryCollection3D& geometry,
                              const Topology3D::Topology3D& topology)
{
    const auto pointWeights = ProtectingBallPlacer::place(discretizationResult, topology, geometry);

    const auto& meshData = context.getMeshData();
    const auto delaunayResult = Delaunay3D::triangulate(context.getOperations(),
                                                        discretizationResult.points,
                                                        discretizationResult.geometryIds,
                                                        pointWeights);

    spdlog::info("RCDTMesher::seedTriangulation: Delaunay3D produced {} nodes, {} elements",
                 meshData.getNodeCount(), meshData.getElementCount());

    context.getMutator().setCurveSegmentManager(
        CurveSegmentBuilder::build(topology, geometry, discretizationResult,
                                   delaunayResult.pointIndexToNodeIdMap));

    spdlog::info("RCDTMesher::seedTriangulation: {} curve segments added",
                 meshData.getCurveSegmentManager().size());
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

void smoothSurfaceMesh(MeshingContext3D& context,
                       const Geometry3D::GeometryCollection3D& geometry,
                       SurfaceMesh3D& surfaceMesh,
                       size_t iterations,
                       bool meshingVolume)
{
    if (iterations == 0)
        return;
    spdlog::info("RCDTMesher: smoothing surface mesh ({} iterations)", iterations);
    // In volume mode the surface nodes are shared with the solid's
    // tetrahedra, which smoothing must not turn inside out.
    std::vector<std::array<size_t, 4>> tetrahedra;
    if (meshingVolume)
        tetrahedra = RCDTMeshExtractor::extractTetrahedra(context.getMeshData());
    SurfaceMeshSmoother::smooth(geometry, surfaceMesh, iterations, tetrahedra);
    syncNodePositions(context, surfaceMesh);
}

// AmbientTetrahedronRemover's flood fill crosses every face that is not
// restricted, so a hole in the restricted boundary lets it walk into the solid
// and delete tetrahedra that belong to the model -- OPE-185's empty BoxWithHole
// mesh. A surface mesh with a hole is still a usable result that reports its
// own defects; a volume mesh missing part of its interior is not.
void requireClosedBoundary(size_t missingFaceEdges)
{
    if (missingFaceEdges == 0)
        return;

    OPENLOOM_THROW_MESH(GENERATION_FAILED,
                        "RCDTMesher::meshVolume: the restricted boundary still has holes (" +
                            std::to_string(missingFaceEdges) +
                            " edges missing a face), so the solid's interior cannot be separated "
                            "from the ambient tetrahedra");
}

// The edges of the restricted boundary missing a face, read against the
// coverage the CAD topology calls for (RestrictedFaceAudit) rather than
// "every edge has two faces". Read-only: the CGAL-style path removes nothing.
size_t countMissingFaceEdges(const RestrictedFaceMap& restrictedFaces,
                             const Topology3D::Topology3D& topology,
                             const MeshData3D& meshData)
{
    const auto defects = RestrictedFaceAudit::findNonManifoldEdges(
        restrictedFaces, RestrictedFaceAudit::buildEdgeToAdjacentSurfaces(topology), meshData);
    return static_cast<size_t>(std::count_if(defects.begin(), defects.end(),
                                             [](const NonManifoldRestrictedEdge& defect)
                                             { return defect.defect == RestrictedEdgeDefect::MissingFace; }));
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
    // A non-positive floor is not "no floor": WeightedDualRestriction sizes
    // its tessellations by it, and SurfaceTessellation builds no cells at all
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
                                      RestrictedFaceMap& restrictedFaces,
                                      bool meshingVolume) const
{
    const double minimumEdgeLength = seedTriangulation(context);

    SurfaceDelaunayRefiner surfaceRefiner(context, *topology_, qualitySettings_, minimumEdgeLength);
    if (meshingVolume)
        TetrahedronDelaunayRefiner(context, surfaceRefiner, qualitySettings_).refine();
    else
        surfaceRefiner.refine();
    restrictedFaces = surfaceRefiner.getRestrictedFaces();

    if (meshingVolume)
        requireClosedBoundary(countMissingFaceEdges(restrictedFaces, *topology_, context.getMeshData()));

    // getOperations()'s mutator, not getMutator(): the latter validates node
    // removal against a MeshConnectivity snapshot that refinement's
    // insertions never refresh.
    AmbientTetrahedronRemover::remove(context.getMeshData(), context.getOperations().getMutator(), restrictedFaces);
    SurfaceMesh3D surfaceMesh = RCDTMeshExtractor::extractSurfaceMesh(context.getMeshData(), restrictedFaces, *topology_);
    smoothSurfaceMesh(context, *geometry_, surfaceMesh, qualitySettings_.smoothingIterations, meshingVolume);
    return surfaceMesh;
}

SurfaceMesh3D RCDTMesher::meshSurface()
{
    MeshingContext3D context(*geometry_, *topology_);
    RestrictedFaceMap restrictedFaces;
    SurfaceMesh3D surfaceMesh = runPipeline(context, restrictedFaces, false);

    // Triangle-only export of the actual output: the restricted facets, none
    // of the ambient tetrahedralization's other faces.
    exportSurfaceMesh3D(surfaceMesh, "rcdt_surface_mesh.vtu");

    return surfaceMesh;
}

VolumeMesh3D RCDTMesher::meshVolume()
{
    MeshingContext3D context(*geometry_, *topology_);
    RestrictedFaceMap restrictedFaces;

    // The returned SurfaceMesh3D is only needed for the smoother's triangle
    // adjacency inside the pipeline — smoothing already synced the resulting
    // positions back into the live mesh, so extractVolumeMesh() (reading that
    // live mesh directly) sees the same, consistent positions.
    runPipeline(context, restrictedFaces, true);
    return RCDTMeshExtractor::extractVolumeMesh(context.getMeshData(), restrictedFaces, *topology_);
}

double RCDTMesher::seedTriangulation(MeshingContext3D& context) const
{
    spdlog::info("RCDTMesher::seedTriangulation: discretizing boundary ({} surface samples/direction)",
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

    spdlog::info("RCDTMesher::seedTriangulation: {} points after discretization",
                 discretizationResult->points.size());

    const double minimumEdgeLength =
        resolveMinimumEdgeLength(qualitySettings_, sizingField ? &sizingField.value() : nullptr,
                                 discretizationResult->points);
    spdlog::info("RCDTMesher::seedTriangulation: minimum edge length = {}", minimumEdgeLength);

    seedAmbientTriangulation(context, *discretizationResult, *geometry_, *topology_);

    return minimumEdgeLength;
}

} // namespace Meshing
