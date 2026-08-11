#include "Meshing/Core/3D/RCDT/RCDTMesher.h"

#include "Common/Exceptions/GeometryException.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Meshing/Core/3D/General/BoundaryDiscretizer3D.h"
#include "Meshing/Core/3D/General/DiscretizationResult3D.h"
#include "Meshing/Core/3D/General/MeshDebugUtils3D.h"
#include "Meshing/Core/3D/General/MeshOperations3D.h"
#include "Meshing/Core/3D/General/MeshingContext3D.h"
#include "Meshing/Core/3D/RCDT/AmbientTetrahedronClassifier.h"
#include "Meshing/Core/3D/RCDT/CurveProtectionSubdivider.h"
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
// the median nearest-neighbor distance among the points, divided by 10.
// Median rather than minimum because a periodic curve's discretization can
// leave a short "remainder" segment near its seam vertex that isn't
// representative of the intended spacing (see project memory: Linear ticket
// on removing OCC seams).
//
// Takes the raw discretization points rather than MeshData3D: computed from
// discretizationResult->points before Delaunay3D::triangulate() runs (so
// CurveProtectionSubdivider has a floor to subdivide against), which is
// exactly the same point set the initial mesh's non-bounding nodes have
// right after triangulation -- inserting the bounding tetrahedron and
// triangulating neither adds nor removes any of them.
double computeMinimumEdgeLength(const std::vector<Point3D>& points)
{
    std::vector<double> nearestPerPoint;
    nearestPerPoint.reserve(points.size());
    for (size_t i = 0; i < points.size(); ++i)
    {
        double nearest = std::numeric_limits<double>::max();
        for (size_t j = 0; j < points.size(); ++j)
        {
            if (i == j)
                continue;
            nearest = std::min(nearest, (points[i] - points[j]).norm());
        }
        nearestPerPoint.push_back(nearest);
    }
    if (nearestPerPoint.empty())
        return 0.0;

    std::sort(nearestPerPoint.begin(), nearestPerPoint.end());
    const double median = nearestPerPoint[nearestPerPoint.size() / 2];
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

    // Resolved here, before CurveProtectionSubdivider/Delaunay3D run, since
    // the subdivider needs a size floor to subdivide against -- see
    // computeMinimumEdgeLength()'s doc for why this is the same value
    // computing it from the post-triangulation mesh would give.
    if (!qualitySettings_.minimumEdgeLength)
        qualitySettings_.minimumEdgeLength = computeMinimumEdgeLength(discretizationResult->points);
    spdlog::info("RCDTMesher::buildInitial: minimum edge length = {}", *qualitySettings_.minimumEdgeLength);

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
    const auto pointWeightsByIndex = CurveProtectionSubdivider::subdivide(
        *discretizationResult, *topology_, *geometry_, *qualitySettings_.minimumEdgeLength);

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

    // Every priority in RCDTRefiner is guarded against inserting a point
    // inside an existing protecting ball (see
    // RCDTRefiner::encroachesProtectingBall()'s doc) -- including priority
    // 1's segment splits via trySplitSegment(), which needs the guard too:
    // a split point lying on the protected curve itself can still land
    // inside an UNRELATED ball (e.g. a nearby corner's), which would
    // otherwise orphan the mesh via the empty-conflict-set fallback in
    // weighted Bowyer-Watson insertion.
    std::vector<double> pointWeights(discretizationResult->points.size(), 0.0);
    for (const auto& [pointIndex, weight] : pointWeightsByIndex)
        pointWeights[pointIndex] = weight;
    Delaunay3D delaunay(meshingContext_->getOperations(), discretizationResult->points, enrichedGeometryIds,
                        pointWeights);
    delaunay.triangulate();
    const auto pointIndexToNodeIdMap = delaunay.getPointIndexToNodeIdMap();
    boundingNodeIds_ = delaunay.getBoundingNodeIds();

    spdlog::info("RCDTMesher::buildInitial: Delaunay3D produced {} nodes, {} elements",
                 meshData.getNodeCount(), meshData.getElementCount());

    // Populated before RestrictedTriangulation::buildFrom() below, not after:
    // classifyFace() consults the CurveSegmentManager to recognize a
    // genuinely protected edge (see its doc), so that lookup needs the curve
    // network in place for the very first classification pass, not just for
    // ones triggered later by refinement.
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

    meshingContext_->rebuildConnectivity();

    restrictedTriangulation_ = std::make_unique<RestrictedTriangulation>();
    const MeshConnectivity connectivity(meshData);
    restrictedTriangulation_->buildFrom(meshData, connectivity, *geometry_, *topology_,
                                        *qualitySettings_.minimumEdgeLength, qualitySettings_);

    spdlog::info("RCDTMesher::buildInitial: {} restricted faces",
                 restrictedTriangulation_->getRestrictedFaces().size());
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
