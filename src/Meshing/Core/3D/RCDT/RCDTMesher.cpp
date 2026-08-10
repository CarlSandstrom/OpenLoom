#include "Meshing/Core/3D/RCDT/RCDTMesher.h"

#include "Common/Exceptions/GeometryException.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Meshing/Core/3D/General/BoundaryDiscretizer3D.h"
#include "Meshing/Core/3D/General/DiscretizationResult3D.h"
#include "Meshing/Core/3D/General/MeshDebugUtils3D.h"
#include "Meshing/Core/3D/General/MeshOperations3D.h"
#include "Meshing/Core/3D/General/MeshingContext3D.h"
#include "Meshing/Core/3D/RCDT/AmbientTetrahedronClassifier.h"
#include "Meshing/Core/3D/RCDT/CurveSegmentOperations.h"
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
#include "Topology/Topology3D.h"
#include "spdlog/spdlog.h"

#include <algorithm>
#include <limits>
#include <unordered_map>

namespace Meshing
{

namespace
{

// Computes a minimum edge length from the initial mesh's node distribution —
// the median nearest-neighbor distance among non-bounding nodes, divided by
// 10. Median rather than minimum because a periodic curve's discretization
// can leave a short "remainder" segment near its seam vertex that isn't
// representative of the intended spacing (see project memory: Linear ticket
// on removing OCC seams).
double computeMinimumEdgeLength(const MeshData3D& meshData)
{
    const auto& nodes = meshData.getNodes();
    const auto& boundingNodeIds = meshData.getBoundingNodeIds();

    const auto isBoundingNode = [&boundingNodeIds](size_t nodeId)
    {
        if (!boundingNodeIds)
            return false;
        for (const size_t id : *boundingNodeIds)
            if (id == nodeId)
                return true;
        return false;
    };

    std::vector<double> nearestPerNode;
    for (const auto& [nodeId, node] : nodes)
    {
        if (isBoundingNode(nodeId))
            continue;
        double nearest = std::numeric_limits<double>::max();
        for (const auto& [otherId, otherNode] : nodes)
        {
            if (otherId == nodeId || isBoundingNode(otherId))
                continue;
            nearest = std::min(nearest, (node->getCoordinates() - otherNode->getCoordinates()).norm());
        }
        nearestPerNode.push_back(nearest);
    }
    if (nearestPerNode.empty())
        return 0.0;

    std::sort(nearestPerNode.begin(), nearestPerNode.end());
    const double median = nearestPerNode[nearestPerNode.size() / 2];
    constexpr double AUTO_MINIMUM_EDGE_LENGTH_DIVISOR = 10.0;
    return median / AUTO_MINIMUM_EDGE_LENGTH_DIVISOR;
}

// Zero-fill rather than plain resize(): node IDs below the highest surviving
// one may be gaps left by the removed supertet corners, and Eigen's default
// constructor does not zero-initialize — an unfilled slot would otherwise
// hold whatever was previously in that memory.
std::vector<Point3D> buildZeroFilledNodeList(const MeshData3D& meshData)
{
    std::vector<Point3D> nodes;
    if (meshData.getNodeCount() == 0)
        return nodes;

    size_t maxNodeId = 0;
    for (const auto& [nodeId, node] : meshData.getNodes())
        maxNodeId = std::max(maxNodeId, nodeId);

    nodes.resize(maxNodeId + 1, Point3D::Zero());
    for (const auto& [nodeId, node] : meshData.getNodes())
        nodes[nodeId] = node->getCoordinates();
    return nodes;
}

} // namespace

RCDTMesher::RCDTMesher(const Geometry3D::GeometryCollection3D& geometry,
                       const Topology3D::Topology3D& topology,
                       Geometry3D::DiscretizationSettings3D discretizationSettings,
                       SurfaceMesh3DQualitySettings qualitySettings) :
    geometry_(&geometry),
    topology_(&topology),
    discretizationSettings_(discretizationSettings),
    qualitySettings_(qualitySettings),
    meshingContext_(std::make_unique<MeshingContext3D>(geometry, topology))
{
}

RCDTMesher::~RCDTMesher() = default;
RCDTMesher::RCDTMesher(RCDTMesher&&) noexcept = default;
RCDTMesher& RCDTMesher::operator=(RCDTMesher&&) noexcept = default;

SurfaceMesh3D RCDTMesher::runPipeline(bool includeTetQualityRefinement)
{
    size_t counter = 0;
    buildInitial();
    Meshing::exportMesh3D(meshingContext_->getMeshData(), "rcdt_initial", counter);
    ++counter;

    refine(includeTetQualityRefinement);
    Meshing::exportMesh3D(meshingContext_->getMeshData(), "rcdt_refined", counter);
    ++counter;

    removeBoundingTetrahedron();

    SurfaceMesh3D surfaceMesh = buildSurfaceMesh();

    if (qualitySettings_.smoothingIterations > 0)
    {
        spdlog::info("RCDTMesher: smoothing surface mesh ({} iterations)",
                     qualitySettings_.smoothingIterations);
        const SurfaceMeshSmoother smoother(*geometry_);
        smoother.smooth(surfaceMesh, qualitySettings_.smoothingIterations);

        // Smoothing only moves the SurfaceMesh3D copy above. The same node IDs
        // are still referenced by the ambient tetrahedra in meshingContext_'s
        // live MeshData3D (buildVolumeMesh() reads those directly) — sync the
        // smoothed positions back so both stay geometrically consistent,
        // rather than only the returned copy.
        auto& mutator = meshingContext_->getMutator();
        for (size_t nodeId = 0; nodeId < surfaceMesh.nodes.size(); ++nodeId)
        {
            if (meshingContext_->getMeshData().getNode(nodeId))
                mutator.moveNode(nodeId, surfaceMesh.nodes[nodeId]);
        }
    }

    Meshing::exportMesh3D(meshingContext_->getMeshData(), "rcdt_smoothed", counter);
    ++counter;

    return surfaceMesh;
}

SurfaceMesh3D RCDTMesher::meshSurface()
{
    SurfaceMesh3D surfaceMesh = runPipeline(false);

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
    // The returned SurfaceMesh3D is only needed for the smoother's triangle
    // adjacency inside runPipeline() — smoothing already synced the resulting
    // positions back into the live mesh, so buildVolumeMesh() (reading that
    // live mesh directly) sees the same, consistent positions.
    runPipeline(true);

    return buildVolumeMesh();
}

const MeshingContext3D& RCDTMesher::getMeshingContext() const
{
    OPENLOOM_REQUIRE_NOT_NULL(meshingContext_.get(), "meshingContext_");
    return *meshingContext_;
}

void RCDTMesher::buildInitial()
{
    spdlog::info("RCDTMesher::buildInitial: discretizing boundary ({} surface samples/direction)",
                 discretizationSettings_.getNumSamplesPerSurfaceDirection());

    BoundaryDiscretizer3D discretizer(*geometry_, *topology_, discretizationSettings_);
    discretizer.discretize();
    auto discretizationResult = discretizer.releaseDiscretizationResult();

    spdlog::info("RCDTMesher::buildInitial: {} points after discretization",
                 discretizationResult->points.size());

    std::unordered_map<std::string, std::vector<std::string>> edgeToAdjacentSurfaces;
    for (const auto& edgeId : topology_->getAllEdgeIds())
        edgeToAdjacentSurfaces[edgeId] = topology_->getEdge(edgeId).getAdjacentSurfaceIds();

    auto enrichedGeometryIds = discretizationResult->geometryIds;
    for (auto& ids : enrichedGeometryIds)
    {
        std::vector<std::string> toAdd;
        for (const auto& geometryId : ids)
        {
            auto it = edgeToAdjacentSurfaces.find(geometryId);
            if (it == edgeToAdjacentSurfaces.end())
                continue;
            for (const auto& surfaceId : it->second)
            {
                if (std::find(ids.begin(), ids.end(), surfaceId) == ids.end() &&
                    std::find(toAdd.begin(), toAdd.end(), surfaceId) == toAdd.end())
                {
                    toAdd.push_back(surfaceId);
                }
            }
        }
        ids.insert(ids.end(), toAdd.begin(), toAdd.end());
    }

    auto& meshData = meshingContext_->getMeshData();

    // NOT YET WIRED IN (OPE-176): CurveProtectionScheme can compute
    // protecting-ball weights for the curve/corner network here, and
    // Delaunay3D/insertVertexBowyerWatson/RCDTRefiner (see
    // RCDTRefiner::encroachesProtectingBall()) correctly persist and act on
    // them end to end. A corners-before-interior sequencing bug in the
    // disjointness clamp (property 2) was found and fixed -- it sized a
    // corner-adjacent interior point's compensation against the corner's
    // PRE-clamp radius, which a later clamp pass could (and did) shrink out
    // from under it. That fix is real and worth keeping, but re-testing on
    // RCDTMesherTorusTest confirmed it doesn't resolve that test's failure:
    // the interior point on the torus's periodic seam has its OWN
    // independently-binding disjointness clamp (an unrelated feature sits
    // close to it directly, not just close to its corner), so no
    // corner-side fix can help -- the clamp numbers logged are bit-for-bit
    // identical before and after the sequencing fix. This is a genuine
    // local-feature-size conflict: the torus's sampling density is too
    // coarse, relative to how close two unrelated parts of its geometry
    // pass to each other, for ANY single-point resizing to satisfy both
    // properties. The real fix is inserting additional points near the
    // tight region so radii can shrink in several smaller geometric steps
    // (the standard Boissonnat-Oudot approach) rather than one large jump --
    // a bigger change (CurveProtectionScheme would need to add points, not
    // just size existing ones) out of scope here. Re-enable this block once
    // that's implemented.
    Delaunay3D delaunay(meshingContext_->getOperations(), discretizationResult->points, enrichedGeometryIds);
    delaunay.triangulate();
    const auto pointIndexToNodeIdMap = delaunay.getPointIndexToNodeIdMap();
    boundingNodeIds_ = delaunay.getBoundingNodeIds();

    spdlog::info("RCDTMesher::buildInitial: Delaunay3D produced {} nodes, {} elements",
                 meshData.getNodeCount(), meshData.getElementCount());

    // Resolve minimumEdgeLength here — the initial mesh is present, so the
    // auto-computation (median nearest-neighbor / 10) has the data it needs.
    // Storing it back into qualitySettings_ means RCDTRefiner reads the same
    // value without recomputing, and the surface tessellation oracle can be
    // sized correctly from the start rather than rebuilt during refinement.
    if (!qualitySettings_.minimumEdgeLength)
        qualitySettings_.minimumEdgeLength = computeMinimumEdgeLength(meshData);
    spdlog::info("RCDTMesher::buildInitial: minimum edge length = {}",
                 *qualitySettings_.minimumEdgeLength);

    meshingContext_->rebuildConnectivity();

    restrictedTriangulation_ = std::make_unique<RestrictedTriangulation>();
    const MeshConnectivity connectivity(meshData);
    restrictedTriangulation_->buildFrom(meshData, connectivity, *geometry_, *topology_,
                                        *qualitySettings_.minimumEdgeLength, qualitySettings_);

    spdlog::info("RCDTMesher::buildInitial: {} restricted faces",
                 restrictedTriangulation_->getRestrictedFaces().size());

    CurveSegmentManager temporarySegmentManager;
    buildCurveSegments(temporarySegmentManager, *topology_, *geometry_,
                       discretizationResult->edgeIdToPointIndicesMap,
                       pointIndexToNodeIdMap,
                       discretizationResult->edgeParameters);

    auto& mutator = meshingContext_->getMutator();
    for (const auto& [segmentId, segment] : temporarySegmentManager.getAllSegments())
        mutator.addCurveSegment(segment);

    spdlog::info("RCDTMesher::buildInitial: {} curve segments added",
                 meshData.getCurveSegmentManager().size());
}

void RCDTMesher::refine(bool includeTetQualityRefinement)
{
    spdlog::info("RCDTMesher::refine: starting RCDT refinement (tet quality: {})",
                 includeTetQualityRefinement);

    std::unique_ptr<RCDTTetQualityController> tetQualityController;
    if (includeTetQualityRefinement)
    {
        tetQualityController =
            std::make_unique<RCDTTetQualityController>(meshingContext_->getMeshData(), qualitySettings_);
    }

    RCDTRefiner refiner(*meshingContext_, *restrictedTriangulation_, qualitySettings_, tetQualityController.get());
    refiner.refine();
    spdlog::info("RCDTMesher::refine: done");
}

void RCDTMesher::removeBoundingTetrahedron()
{
    // Strips every ambient tetrahedron -- both the seed triangulation's
    // outer shell (touching the supertet's corners) and, for domains with
    // holes, the tets RCDT kept triangulating interior voids with -- not
    // just the ones literally touching a bounding node. See
    // AmbientTetrahedronClassifier's class docs for why one flood fill
    // handles both.
    AmbientTetrahedronClassifier ambientClassifier;
    ambientClassifier.classify(meshingContext_->getMeshData(), *restrictedTriangulation_);

    // getOperations()'s mutator, not getMutator(): the latter validates node
    // removal against a MeshConnectivity snapshot that's only refreshed by an
    // explicit rebuildConnectivity() call, and refine()'s many insertions
    // never call it -- that snapshot is stale by the time we get here. The
    // operations mutator performs no such (now-stale) validation.
    auto& mutator = meshingContext_->getOperations().getMutator();
    const auto& meshData = meshingContext_->getMeshData();

    std::vector<size_t> ambientTetIds;
    for (const auto& [elementId, element] : meshData.getElements())
    {
        if (dynamic_cast<const TetrahedralElement*>(element.get()) && ambientClassifier.isAmbient(elementId))
            ambientTetIds.push_back(elementId);
    }

    for (const size_t tetId : ambientTetIds)
        mutator.removeElement(tetId);

    for (const size_t nodeId : boundingNodeIds_)
        mutator.removeNode(nodeId);
    mutator.clearBoundingNodeIds();

    spdlog::info("RCDTMesher::removeBoundingTetrahedron: Removed {} ambient tetrahedra "
                 "(true exterior + holes) and 4 bounding nodes",
                 ambientTetIds.size());

    meshingContext_->rebuildConnectivity();
}

SurfaceMesh3D RCDTMesher::buildSurfaceMesh() const
{
    SurfaceMesh3D surfaceMesh;
    const auto& meshData = meshingContext_->getMeshData();

    surfaceMesh.nodes = buildZeroFilledNodeList(meshData);

    for (const auto& [faceKey, surfaceId] : restrictedTriangulation_->getRestrictedFaces())
    {
        const size_t triangleIndex = surfaceMesh.triangles.size();
        surfaceMesh.triangles.push_back({faceKey.nodeIds[0], faceKey.nodeIds[1], faceKey.nodeIds[2]});
        surfaceMesh.faceTriangleIds[surfaceId].push_back(triangleIndex);
    }

    const auto& curveSegmentManager = meshData.getCurveSegmentManager();
    for (const auto& edgeId : topology_->getAllEdgeIds())
    {
        const auto segments = curveSegmentManager.getSegmentsForEdge(edgeId);
        if (segments.empty())
            continue;

        std::vector<size_t> nodeIds;
        nodeIds.push_back(segments[0].nodeId1);
        for (const auto& segment : segments)
            nodeIds.push_back(segment.nodeId2);

        surfaceMesh.edgeNodeIds[edgeId] = std::move(nodeIds);
    }

    spdlog::debug("RCDTMesher::buildSurfaceMesh: {} nodes, {} triangles, {} faces, {} edges",
                  surfaceMesh.nodes.size(), surfaceMesh.triangles.size(),
                  surfaceMesh.faceTriangleIds.size(), surfaceMesh.edgeNodeIds.size());

    return surfaceMesh;
}

VolumeMesh3D RCDTMesher::buildVolumeMesh() const
{
    VolumeMesh3D volumeMesh;
    const auto& meshData = meshingContext_->getMeshData();

    volumeMesh.nodes = buildZeroFilledNodeList(meshData);

    // removeBoundingTetrahedron() already ran (part of runPipeline(), called
    // before this) — every tetrahedron still in meshData is a genuine
    // interior tet, none touch the supertet's corners, so no extra filtering
    // is needed here.
    for (const auto& [elementId, element] : meshData.getElements())
    {
        const auto* tet = dynamic_cast<const TetrahedralElement*>(element.get());
        if (!tet)
            continue;

        const auto& nodeIds = tet->getNodeIds();
        volumeMesh.tetrahedra.push_back({nodeIds[0], nodeIds[1], nodeIds[2], nodeIds[3]});
    }

    for (const auto& [faceKey, surfaceId] : restrictedTriangulation_->getRestrictedFaces())
    {
        const size_t triangleIndex = volumeMesh.boundaryTriangles.size();
        volumeMesh.boundaryTriangles.push_back({faceKey.nodeIds[0], faceKey.nodeIds[1], faceKey.nodeIds[2]});
        volumeMesh.boundaryFaceTriangleIds[surfaceId].push_back(triangleIndex);
    }

    const auto& curveSegmentManager = meshData.getCurveSegmentManager();
    for (const auto& edgeId : topology_->getAllEdgeIds())
    {
        const auto segments = curveSegmentManager.getSegmentsForEdge(edgeId);
        if (segments.empty())
            continue;

        std::vector<size_t> nodeIds;
        nodeIds.push_back(segments[0].nodeId1);
        for (const auto& segment : segments)
            nodeIds.push_back(segment.nodeId2);

        volumeMesh.boundaryEdgeNodeIds[edgeId] = std::move(nodeIds);
    }

    spdlog::debug("RCDTMesher::buildVolumeMesh: {} nodes, {} tetrahedra, {} boundary triangles, "
                  "{} boundary faces, {} boundary edges",
                  volumeMesh.nodes.size(), volumeMesh.tetrahedra.size(), volumeMesh.boundaryTriangles.size(),
                  volumeMesh.boundaryFaceTriangleIds.size(), volumeMesh.boundaryEdgeNodeIds.size());

    return volumeMesh;
}

} // namespace Meshing
