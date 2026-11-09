#include "Meshing/Core/3D/RCDT/RCDTMesher.h"

#include "Common/Exceptions/MeshException.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Meshing/Core/3D/General/BoundaryDiscretizer3D.h"
#include "Meshing/Core/3D/General/MeshDebugUtilities3D.h"
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
double resolveMinimumEdgeLength(const SurfaceMesh3DQualitySettings& qualitySettings,
                                const std::optional<SizingField3D>& sizingField,
                                const std::vector<Point3D>& points);
void seedAmbientTriangulation(MeshingContext3D& context,
                              DiscretizationResult3D& discretizationResult,
                              const Geometry3D::GeometryCollection3D& geometry,
                              const Topology3D::Topology3D& topology);
void smoothSurfaceMesh(MeshingContext3D& context,
                       const Geometry3D::GeometryCollection3D& geometry,
                       SurfaceMesh3D& surfaceMesh,
                       size_t iterations,
                       bool meshingVolume);
void requireClosedBoundary(size_t missingFaceEdges);
size_t countMissingFaceEdges(const RestrictedFaceMap& restrictedFaces,
                             const Topology3D::Topology3D& topology,
                             const MeshData3D& meshData);
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
    // A non-positive floor is not "no floor": SurfaceTessellation builds no cells
    // for a target size <= 0, leaving the crossing test nothing to test against.
    auto minimumEdgeLength = qualitySettings_.minimumEdgeLength;
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
    double minimumEdgeLength = seedTriangulation(context);

    SurfaceDelaunayRefiner surfaceRefiner(context, *topology_, qualitySettings_, minimumEdgeLength);
    if (meshingVolume)
        TetrahedronDelaunayRefiner(context, surfaceRefiner, qualitySettings_).refine();
    else
        surfaceRefiner.refine();
    restrictedFaces = surfaceRefiner.getRestrictedFaces();

    if (meshingVolume)
        requireClosedBoundary(countMissingFaceEdges(restrictedFaces, *topology_, context.getMeshData()));

    // getOperations()'s mutator, not getMutator(): the latter validates removal
    // against a MeshConnectivity snapshot that refinement never refreshes.
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

    exportSurfaceMesh3D(surfaceMesh, "rcdt_surface_mesh.vtu");

    return surfaceMesh;
}

VolumeMesh3D RCDTMesher::meshVolume()
{
    MeshingContext3D context(*geometry_, *topology_);
    RestrictedFaceMap restrictedFaces;

    // The returned surface mesh is not needed: smoothing already synced its
    // positions back into the live mesh that extractVolumeMesh() reads.
    runPipeline(context, restrictedFaces, true);
    return RCDTMeshExtractor::extractVolumeMesh(context.getMeshData(), restrictedFaces, *topology_);
}

double RCDTMesher::seedTriangulation(MeshingContext3D& context) const
{
    spdlog::info("RCDTMesher::seedTriangulation: discretizing boundary ({} surface samples/direction)",
                 discretizationSettings_.getNumberOfSamplesPerSurfaceDirection());

    // Kept, so the size floor reads the same h(x) the discretization did.
    std::optional<SizingField3D> sizingField;
    if (sizingFieldSettings_.has_value())
        sizingField = SizingFieldBuilder3D::build(*geometry_, *topology_, sizingFieldSettings_.value());

    auto discretizationResult =
        BoundaryDiscretizer3D::discretize(*geometry_, *topology_, discretizationSettings_, sizingField);

    spdlog::info("RCDTMesher::seedTriangulation: {} points after discretization",
                 discretizationResult->points.size());

    double minimumEdgeLength = resolveMinimumEdgeLength(qualitySettings_, sizingField, discretizationResult->points);
    spdlog::info("RCDTMesher::seedTriangulation: minimum edge length = {}", minimumEdgeLength);

    seedAmbientTriangulation(context, *discretizationResult, *geometry_, *topology_);

    return minimumEdgeLength;
}

namespace
{

// The size floor: SurfaceTessellation's cell size in WeightedDualRestriction.
double resolveMinimumEdgeLength(const SurfaceMesh3DQualitySettings& qualitySettings,
                                const std::optional<SizingField3D>& sizingField,
                                const std::vector<Point3D>& points)
{
    if (qualitySettings.minimumEdgeLength) return *qualitySettings.minimumEdgeLength;
    if (sizingField) return MinimumEdgeLengthEstimator::fromSizingField(*sizingField);
    return MinimumEdgeLengthEstimator::fromPointSpacing(points);
}

// Boissonnat-Oudot protecting balls: corner and curve points enter the initial
// triangulation as weighted points, so every crease appears as an edge chain.
void seedAmbientTriangulation(MeshingContext3D& context,
                              DiscretizationResult3D& discretizationResult,
                              const Geometry3D::GeometryCollection3D& geometry,
                              const Topology3D::Topology3D& topology)
{
    auto pointWeights = ProtectingBallPlacer::place(discretizationResult, topology, geometry);

    auto& meshData = context.getMeshData();
    auto delaunayResult = Delaunay3D::triangulate(context.getOperations(),
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

// Smoothing only moves the SurfaceMesh3D copy, but extractVolumeMesh() reads the
// live MeshData3D, so the smoothed positions are synced back.
void synchronizeNodePositions(MeshingContext3D& context, const SurfaceMesh3D& surfaceMesh)
{
    auto& mutator = context.getMutator();
    for (size_t nodeId = 0; nodeId < surfaceMesh.nodes.size(); ++nodeId)
    {
        if (context.getMeshData().getNode(nodeId)) mutator.moveNode(nodeId, surfaceMesh.nodes[nodeId]);
    }
}

void smoothSurfaceMesh(MeshingContext3D& context,
                       const Geometry3D::GeometryCollection3D& geometry,
                       SurfaceMesh3D& surfaceMesh,
                       size_t iterations,
                       bool meshingVolume)
{
    if (iterations == 0) return;

    spdlog::info("RCDTMesher: smoothing surface mesh ({} iterations)", iterations);
    // In volume mode smoothing must not turn the solid's tetrahedra inside out.
    std::vector<std::array<size_t, 4>> tetrahedra;
    if (meshingVolume)
        tetrahedra = RCDTMeshExtractor::extractTetrahedra(context.getMeshData());
    SurfaceMeshSmoother::smooth(geometry, surfaceMesh, iterations, tetrahedra);
    synchronizeNodePositions(context, surfaceMesh);
}

// A hole in the restricted boundary lets AmbientTetrahedronRemover's flood fill
// walk into the solid and delete its tetrahedra (OPE-185). A surface mesh with a
// hole still reports its own defects; a gutted volume mesh does not.
void requireClosedBoundary(size_t missingFaceEdges)
{
    if (missingFaceEdges == 0) return;

    OPENLOOM_THROW_MESH(GENERATION_FAILED,
                        "RCDTMesher::meshVolume: the restricted boundary still has holes (" +
                            std::to_string(missingFaceEdges) +
                            " edges missing a face), so the solid's interior cannot be separated "
                            "from the ambient tetrahedra");
}

// Edges missing a face, judged against the coverage the CAD topology calls for
// (RestrictedFaceAudit) rather than "every edge has two faces".
size_t countMissingFaceEdges(const RestrictedFaceMap& restrictedFaces,
                             const Topology3D::Topology3D& topology,
                             const MeshData3D& meshData)
{
    auto defects = RestrictedFaceAudit::findNonManifoldEdges(
        restrictedFaces, RestrictedFaceAudit::buildEdgeToAdjacentSurfaces(topology), meshData);
    return static_cast<size_t>(std::count_if(defects.begin(), defects.end(),
                                             [](const NonManifoldRestrictedEdge& defect)
                                             { return defect.defect == RestrictedEdgeDefect::MissingFace; }));
}

} // namespace

} // namespace Meshing
