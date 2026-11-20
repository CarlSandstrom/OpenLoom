#include "Meshing/Core/3D/RCDT/RCDTMesher.h"

#include "Common/Exceptions/MeshException.h"
#include "Export/VtkGrid.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Meshing/Core/3D/General/BoundaryDiscretizer3D.h"
#include "Meshing/Core/3D/General/MeshOperations3D.h"
#include "Meshing/Core/3D/General/MeshSnapshotWriter.h"
#include "Meshing/Core/3D/General/MeshingContext3D.h"
#include "Meshing/Core/3D/General/SizingField3D.h"
#include "Meshing/Core/3D/General/SurfaceMeshSmoother.h"
#include "Meshing/Core/3D/General/UnreferencedNodeRemover.h"
#include "Meshing/Core/3D/RCDT/AmbientTetrahedronRemover.h"
#include "Meshing/Core/3D/RCDT/CurveSegmentBuilder.h"
#include "Meshing/Core/3D/RCDT/MinimumEdgeLengthEstimator.h"
#include "Meshing/Core/3D/RCDT/ProtectingBallPlacer.h"
#include "Meshing/Core/3D/RCDT/SliverExuder.h"
#include "Meshing/Core/3D/RCDT/RCDTMeshExtractor.h"
#include "Meshing/Core/3D/RCDT/RestrictedFaceAudit.h"
#include "Meshing/Core/3D/RCDT/SnapshotGridBuilder.h"
#include "Meshing/Core/3D/RCDT/SurfaceDelaunayRefiner.h"
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
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace Meshing
{

namespace
{
double resolveMinimumEdgeLength(const SurfaceMesh3DQualitySettings& qualitySettings,
                                const std::optional<SizingField3D>& sizingField,
                                const std::vector<Point3D>& points);
void buildAmbientTriangulation(MeshingContext3D& context,
                               DiscretizationResult3D& discretizationResult,
                               const Geometry3D::GeometryCollection3D& geometry,
                               const Topology3D::Topology3D& topology);
void smoothSurfaceMesh(MeshingContext3D& context,
                       const Geometry3D::GeometryCollection3D& geometry,
                       SurfaceMesh3D& surfaceMesh,
                       size_t iterations,
                       double sliverAngleDegrees,
                       bool meshingVolume);
RestrictedFaceMap refineSurface(MeshingContext3D& context,
                                const Topology3D::Topology3D& topology,
                                const SurfaceMesh3DQualitySettings& qualitySettings,
                                double minimumEdgeLength);
RestrictedFaceMap refineVolume(MeshingContext3D& context,
                               const Topology3D::Topology3D& topology,
                               const SurfaceMesh3DQualitySettings& qualitySettings,
                               double minimumEdgeLength);
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
    discretizationSettings_(std::move(discretizationSettings)),
    qualitySettings_(std::move(qualitySettings)),
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

void RCDTMesher::meshSurface()
{
    surfaceMesh_.reset();
    volumeMesh_.reset();

    MeshSnapshotWriter::restartSequence();
    MeshingContext3D context(*geometry_, *topology_);
    double minimumEdgeLength = buildInitialTriangulation(context);
    auto restrictedFaces = refineSurface(context, *topology_, qualitySettings_, minimumEdgeLength);
    buildSurfaceMesh(context, restrictedFaces, false);
}

void RCDTMesher::meshVolume()
{
    surfaceMesh_.reset();
    volumeMesh_.reset();

    MeshSnapshotWriter::restartSequence();
    MeshingContext3D context(*geometry_, *topology_);
    double minimumEdgeLength = buildInitialTriangulation(context);
    auto restrictedFaces = refineVolume(context, *topology_, qualitySettings_, minimumEdgeLength);
    // Meshing step 7, first part: the closed-boundary check
    requireClosedBoundary(countMissingFaceEdges(restrictedFaces, *topology_, context.getMeshData()));
    buildSurfaceMesh(context, restrictedFaces, true);

    // Smoothing has already synced its positions back into the live mesh that
    // extractVolumeMesh() reads.
    volumeMesh_ = std::make_unique<VolumeMesh3D>(
        RCDTMeshExtractor::extractVolumeMesh(context.getMeshData(), restrictedFaces, *topology_));
    UnreferencedNodeRemover::remove(*volumeMesh_);
}

const SurfaceMesh3D* RCDTMesher::getSurfaceMesh() const
{
    return surfaceMesh_.get();
}

const VolumeMesh3D* RCDTMesher::getVolumeMesh() const
{
    return volumeMesh_.get();
}

void RCDTMesher::buildSurfaceMesh(MeshingContext3D& context,
                                  const RestrictedFaceMap& restrictedFaces,
                                  bool meshingVolume)
{
    // Meshing step 7: remove the ambient tetrahedra (for a volume, meshVolume() has
    // already checked that the boundary is closed).
    AmbientTetrahedronRemover::remove(context, restrictedFaces);

    // Meshing step 8: extract the surface mesh
    surfaceMesh_ = std::make_unique<SurfaceMesh3D>(
        RCDTMeshExtractor::extractSurfaceMesh(context.getMeshData(), restrictedFaces, *topology_));
    // Meshing step 9: smooth
    smoothSurfaceMesh(context, *geometry_, *surfaceMesh_, qualitySettings_.smoothingIterations,
                      qualitySettings_.sliverDihedralAngleDegrees, meshingVolume);
    // Smoothing writes back to the live mesh by node ID, so the output is only
    // renumbered once it is done.
    UnreferencedNodeRemover::remove(*surfaceMesh_);
    if (qualitySettings_.smoothingIterations > 0 && MeshSnapshotWriter::isEnabled())
        MeshSnapshotWriter::write("smoothed", SnapshotGridBuilder::fromSurfaceMesh(*surfaceMesh_));
}

double RCDTMesher::buildInitialTriangulation(MeshingContext3D& context) const
{
    spdlog::info("RCDTMesher::buildInitialTriangulation: discretizing boundary ({} surface samples/direction)",
                 discretizationSettings_.getNumberOfSamplesPerSurfaceDirection());

    // Kept, so the size floor reads the same h(x) the discretization did.
    std::optional<SizingField3D> sizingField;
    if (sizingFieldSettings_.has_value())
    {
        sizingField = SizingFieldBuilder3D::build(*geometry_, *topology_, sizingFieldSettings_.value());
    }

    // Meshing step 1: sample the corners and curves (doc/Meshing_Walkthrough.md)
    auto discretizationResult = BoundaryDiscretizer3D::discretize(*geometry_, *topology_, discretizationSettings_, sizingField);

    spdlog::info("RCDTMesher::buildInitialTriangulation: {} points after discretization", discretizationResult->points.size());

    double minimumEdgeLength = resolveMinimumEdgeLength(qualitySettings_, sizingField, discretizationResult->points);
    spdlog::info("RCDTMesher::buildInitialTriangulation: minimum edge length = {}", minimumEdgeLength);

    buildAmbientTriangulation(context, *discretizationResult, *geometry_, *topology_);

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
void buildAmbientTriangulation(MeshingContext3D& context,
                               DiscretizationResult3D& discretizationResult,
                               const Geometry3D::GeometryCollection3D& geometry,
                               const Topology3D::Topology3D& topology)
{
    // Meshing step 2: place the protecting balls
    auto pointWeights = ProtectingBallPlacer::place(discretizationResult, topology, geometry);

    // Meshing step 3: build the weighted Delaunay tetrahedralization, then record the curve segments
    auto& meshData = context.getMeshData();
    auto delaunayResult = Delaunay3D::triangulate(context.getOperations(),
                                                  discretizationResult.points,
                                                  discretizationResult.geometryIds,
                                                  pointWeights);

    spdlog::info("RCDTMesher::buildInitialTriangulation: Delaunay3D produced {} nodes, {} elements",
                 meshData.getNodeCount(), meshData.getElementCount());

    context.getMutator().setCurveSegmentManager(
        CurveSegmentBuilder::build(topology, geometry, discretizationResult,
                                   delaunayResult.pointIndexToNodeIdMap));

    spdlog::info("RCDTMesher::buildInitialTriangulation: {} curve segments added",
                 meshData.getCurveSegmentManager().size());
}

// Smoothing only moves the SurfaceMesh3D copy, but extractVolumeMesh() reads the
// live MeshData3D, so the smoothed positions are synced back.
void synchronizeNodePositions(MeshingContext3D& context, const SurfaceMesh3D& surfaceMesh)
{
    auto& mutator = context.getMutator();
    for (size_t nodeIndex = 0; nodeIndex < surfaceMesh.nodes.size(); ++nodeIndex)
    {
        NodeId nodeId{nodeIndex};
        if (context.getMeshData().getNode(nodeId)) mutator.moveNode(nodeId, surfaceMesh.nodes[nodeIndex]);
    }
}

void smoothSurfaceMesh(MeshingContext3D& context,
                       const Geometry3D::GeometryCollection3D& geometry,
                       SurfaceMesh3D& surfaceMesh,
                       size_t iterations,
                       double sliverAngleDegrees,
                       bool meshingVolume)
{
    if (iterations == 0) return;

    spdlog::info("RCDTMesher: smoothing surface mesh ({} iterations)", iterations);
    // In volume mode smoothing must not turn the solid's tetrahedra inside out.
    std::vector<std::array<NodeId, 4>> tetrahedra;
    if (meshingVolume) tetrahedra = RCDTMeshExtractor::extractTetrahedra(context.getMeshData());
    SurfaceMeshSmoother::smooth(geometry, surfaceMesh, iterations, tetrahedra, sliverAngleDegrees);
    synchronizeNodePositions(context, surfaceMesh);
}

RestrictedFaceMap refineSurface(MeshingContext3D& context,
                                const Topology3D::Topology3D& topology,
                                const SurfaceMesh3DQualitySettings& qualitySettings,
                                double minimumEdgeLength)
{
    // Meshing steps 4-5: classify the faces, then refine the bad surface facets
    SurfaceDelaunayRefiner refiner(context, topology, qualitySettings, minimumEdgeLength);
    refiner.refine();
    refiner.writeSnapshot("refined");
    return refiner.getRestrictedFaces();
}

RestrictedFaceMap refineVolume(MeshingContext3D& context,
                               const Topology3D::Topology3D& topology,
                               const SurfaceMesh3DQualitySettings& qualitySettings,
                               double minimumEdgeLength)
{
    // Meshing steps 4-6: classify the faces, then alternate between refining bad
    // surface facets (step 5) and bad interior tetrahedra (step 6)
    TetrahedronDelaunayRefiner refiner(context, topology, qualitySettings, minimumEdgeLength);
    refiner.refine();
    auto restrictedFaces = refiner.getRestrictedFaces();
    SliverExuder(context, refiner.getRestriction(), restrictedFaces, qualitySettings.sliverDihedralAngleDegrees).exude();
    return restrictedFaces;
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
