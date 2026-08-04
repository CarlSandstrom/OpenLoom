#include "Meshing/Core/3D/RCDT/RCDTRefiner.h"

#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/IEdge3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Core/3D/General/ElementGeometry3D.h"
#include "Meshing/Core/3D/General/MeshDebugUtils3D.h"
#include "Meshing/Core/3D/General/MeshOperations3D.h"
#include "Meshing/Core/3D/General/MeshQueries3D.h"
#include "Meshing/Core/3D/General/MeshingContext3D.h"
#include "Meshing/Core/3D/RCDT/AmbientTetrahedronClassifier.h"
#include "Meshing/Core/3D/RCDT/CurveSegmentOperations.h"
#include "Meshing/Core/3D/RCDT/RCDTTetQualityController.h"
#include "Meshing/Core/3D/RCDT/RestrictedTriangulation.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Meshing/Data/3D/Node3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"
#include "Meshing/Data/Base/MeshConnectivity.h"
#include "Meshing/Data/CurveSegmentManager.h"
#include "spdlog/spdlog.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <unordered_map>
#include <unordered_set>

namespace Meshing
{

namespace
{

constexpr size_t MAX_ITERATIONS = 500;

// Divisor applied to the initial discretization's smallest pairwise distance
// when settings_.minimumEdgeLength is unset. See resolveMinimumEdgeLength().
constexpr double AUTO_MINIMUM_EDGE_LENGTH_DIVISOR = 10.0;

} // namespace

RCDTRefiner::RCDTRefiner(MeshingContext3D& context,
                         RestrictedTriangulation& restrictedTriangulation,
                         const SurfaceMesh3DQualitySettings& settings,
                         const RCDTTetQualityController* tetQualityController) :
    context_(&context),
    restrictedTriangulation_(&restrictedTriangulation),
    settings_(settings),
    tetQualityController_(tetQualityController)
{
}

void RCDTRefiner::refine()
{
    const auto& meshData = context_->getMeshData();
    spdlog::info("RCDTRefiner: starting refinement — {} nodes, {} segments",
                 meshData.getNodeCount(),
                 meshData.getCurveSegmentManager().size());

    minimumEdgeLength_ = resolveMinimumEdgeLength();
    spdlog::info("RCDTRefiner: minimum edge length floor = {}", minimumEdgeLength_);

    size_t iteration = 0;
    exportMesh3D(context_->getMeshData(), "rcdt_refinement_step", iteration);
    ++iteration;

    while (iteration < MAX_ITERATIONS)
    {
        if (!refineStep()) break;
        exportMesh3D(context_->getMeshData(), "rcdt_refinement_step", iteration);
        ++iteration;
    }

    if (iteration >= MAX_ITERATIONS)
        spdlog::warn("RCDTRefiner: reached iteration cap ({})", MAX_ITERATIONS);

    spdlog::info("RCDTRefiner: done after {} iterations — {} nodes",
                 iteration,
                 context_->getMeshData().getNodeCount());
}

bool RCDTRefiner::refineStep()
{
    const auto& meshData = context_->getMeshData();
    const auto& curveSegmentManager = meshData.getCurveSegmentManager();

    // ---- Priority 1: encroached curve segments ----

    const auto nodePositionMap = buildNodePositionMap();

    std::unordered_set<size_t> encroached;
    for (const auto& [nodeId, node] : meshData.getNodes())
    {
        for (const size_t segmentId :
             curveSegmentManager.findEncroached(node->getCoordinates(), nodePositionMap, nodeId))
        {
            encroached.insert(segmentId);
        }
    }

    for (const size_t segmentId : encroached)
    {
        if (unrefinableSegments_.count(segmentId))
            continue;

        // Size floor (same reasoning as the restricted-triangle/tetrahedron
        // versions below): a segment already at or below the minimum useful
        // element size is left encroached rather than bisected forever.
        const CurveSegment segment = curveSegmentManager.getSegment(segmentId);
        const auto it1 = nodePositionMap.find(segment.nodeId1);
        const auto it2 = nodePositionMap.find(segment.nodeId2);
        const double length = (it1 != nodePositionMap.end() && it2 != nodePositionMap.end())
                                   ? (it1->second - it2->second).norm()
                                   : 0.0;
        if (length <= minimumEdgeLength_)
        {
            unrefinableSegments_.insert(segmentId);
            continue;
        }

        return splitSegment(segmentId);
    }

    // ---- Priority 2: bad restricted triangles ----

    const auto* geometry = context_->getGeometry();
    if (!geometry)
        return false;

    const MeshConnectivity connectivity(meshData);
    const auto badTriangles =
        restrictedTriangulation_->getBadTriangles(settings_, meshData, connectivity, *geometry);

    if (badTriangles.empty())
        return refineRemainingPriorities();

    for (const auto& bad : badTriangles)
    {
        if (unrefinableTriangles_.count(bad.face))
            continue;

        // Size floor (CGAL/Boissonnat-Oudot style): a triangle already at or
        // below the minimum useful element size is left as-is even if still
        // quality-bad, rather than trying (and failing) to fix it forever.
        // Checked before doing any surface work since it needs nothing but
        // the triangle's own shortest edge.
        if (bad.shortestEdge <= minimumEdgeLength_)
        {
            unrefinableTriangles_.insert(bad.face);
            continue;
        }

        const Geometry3D::ISurface3D* surface = geometry->getSurface(bad.surfaceId);
        if (!surface)
        {
            unrefinableTriangles_.insert(bad.face);
            continue;
        }

        // Prefer the true restricted Voronoi vertex (where the face's dual
        // Voronoi edge crosses the surface, already computed by
        // getBadTriangles); fall back to projecting the flat circumcenter if
        // that couldn't be computed — most commonly because the face's
        // classification into restrictedFaces_ has gone stale (an earlier,
        // different pair of neighboring tetrahedra is what originally found
        // a crossing) and its current dual edge no longer crosses the
        // surface at all. The proximity guard below is what actually keeps
        // this fallback safe: it rejects a fallback candidate that lands too
        // close to an unrelated vertex, so a bad fallback point just leaves
        // the triangle unrefined (same outcome as not attempting it), while
        // a good one — which is the common case — still gets used. Removing
        // the fallback entirely was tried and measured worse: many stale
        // classifications still yield a perfectly usable fallback point, and
        // giving up on all of them loses far more good insertions than the
        // rare bad one the guard would have caught anyway.
        std::optional<Point3D> projectedOpt = bad.insertionPoint;
        if (!projectedOpt)
            projectedOpt = surfaceProjector_.projectToSurface(bad.circumcircleCenter, *surface);
        if (!projectedOpt)
        {
            unrefinableTriangles_.insert(bad.face);
            continue;
        }

        const Point3D& projected = *projectedOpt;

        // Proximity guard: reject if the candidate would land within the size
        // floor of any existing vertex, regardless of that vertex's history.
        // Needed in addition to the shortestEdge check above: that check only
        // looks at the triangle's edges *before* insertion, so it cannot
        // catch a circumcenter that projects onto (or nearly onto) an
        // existing vertex — including one belonging to a different, unrelated
        // triangle. Without this, such an insertion creates a near-degenerate
        // duplicate point that corrupts the local mesh and never stops
        // generating new "bad" triangles around it.
        bool tooClose = false;
        for (const auto& [nodeId, node] : meshData.getNodes())
        {
            if ((projected - node->getCoordinates()).norm() < minimumEdgeLength_)
            {
                tooClose = true;
                break;
            }
        }
        if (tooClose)
        {
            unrefinableTriangles_.insert(bad.face);
            continue;
        }

        // Demotion: if the circumcenter would encroach a segment, split that segment instead.
        const auto encroachingIds = curveSegmentManager.findEncroached(projected, nodePositionMap);
        if (!encroachingIds.empty())
            return splitSegment(encroachingIds[0]);

        // Insert the projected circumcenter.
        // Do NOT clear unrefinableTriangles_ here: faces blocked by the size
        // floor stay blocked (they can't get any smaller than they already
        // are). Clearing was the cascade bug — it caused those faces to be
        // retried forever after each subsequent insertion cleared the set.
        // Clearing on segment splits (in splitSegment()) is still correct
        // because splitting a segment changes the constraint structure and
        // may unblock previously stuck faces.
        insertAndUpdate(projected, {bad.surfaceId});
        return true;
    }

    return refineRemainingPriorities();
}

bool RCDTRefiner::refineRemainingPriorities()
{
    if (refineBadTetrahedra())
        return true;
    return refineNonManifoldEdges();
}

bool RCDTRefiner::refineBadTetrahedra()
{
    if (!tetQualityController_)
        return false;

    const auto& meshData = context_->getMeshData();
    const auto& curveSegmentManager = meshData.getCurveSegmentManager();
    const auto nodePositionMap = buildNodePositionMap();

    // Priority 3 runs before removeBoundingTetrahedron() -- priorities 1/2
    // need the supertet kept alive (see class docs) -- so at this point the
    // mesh still contains ambient tetrahedra: the seed triangulation's
    // artifacts touching the supertet's corners, and (for domains with
    // holes) tets filling interior voids that RCDT also keeps triangulated
    // throughout refinement. Neither is real output -- both are naturally
    // "skinny" relative to the real geometry (the supertet is deliberately
    // huge; a hole's tets span an unconstrained gap) -- so attempting to
    // refine them wastes every iteration on circumcenters that are
    // meaningless (or, for a hole, land in space the final mesh won't even
    // contain). AmbientTetrahedronClassifier excludes both in one pass: see
    // its class docs for why a hole's interior and the true exterior are
    // indistinguishable to the ambient tetrahedralization.
    AmbientTetrahedronClassifier ambientClassifier;
    ambientClassifier.classify(meshData, *restrictedTriangulation_);

    const auto skinnyTetIds =
        context_->getOperations().getQueries().findSkinnyTetrahedra(settings_.tetCircumradiusToShortestEdgeRatio);

    const ElementGeometry3D elementGeometry(meshData);

    for (const size_t tetId : skinnyTetIds)
    {
        if (unrefinableTetrahedra_.count(tetId))
            continue;

        const auto* element = meshData.getElement(tetId);
        const auto* tet = dynamic_cast<const TetrahedralElement*>(element);
        if (!tet)
            continue;

        if (ambientClassifier.isAmbient(tetId))
            continue;

        // Size floor, same reasoning as the restricted-triangle version above.
        if (tetQualityController_->isTetrahedronTooSmall(*tet))
        {
            unrefinableTetrahedra_.insert(tetId);
            continue;
        }

        const auto circumsphere = elementGeometry.computeCircumscribingSphere(*tet);
        if (!circumsphere)
        {
            unrefinableTetrahedra_.insert(tetId);
            continue;
        }

        // Sanity guard: computeCircumscribingSphere solves a 3x3 linear
        // system, and a thin/near-flat tetrahedron can have a genuinely huge
        // true circumradius (this is real math, not just numerical error —
        // as a tet flattens, its circumcenter recedes toward infinity) — but
        // that doesn't make inserting it a sensible refinement move: the
        // resulting point can land far outside the region the mesh actually
        // occupies (confirmed empirically: radius 126 for a unit-scale box),
        // corrupting the mesh's extent and never converging. Bound against
        // minimumEdgeLength_ (the mesh's own characteristic scale, not the
        // individual tet's potentially-tiny diameter) rather than trying to
        // fix the circumcenter computation itself — a genuine, hard "sliver"
        // problem general Delaunay refinement is known not to fully solve
        // (see the class doc comment above on slivers).
        constexpr double MAX_CIRCUMRADIUS_TO_MIN_EDGE_LENGTH_RATIO = 100.0;
        if (circumsphere->radius > MAX_CIRCUMRADIUS_TO_MIN_EDGE_LENGTH_RATIO * minimumEdgeLength_)
        {
            unrefinableTetrahedra_.insert(tetId);
            continue;
        }

        const Point3D& circumcenter = circumsphere->center;

        // Proximity guard, same reasoning as the restricted-triangle version above.
        bool tooClose = false;
        for (const auto& [nodeId, node] : meshData.getNodes())
        {
            if ((circumcenter - node->getCoordinates()).norm() < minimumEdgeLength_)
            {
                tooClose = true;
                break;
            }
        }
        if (tooClose)
        {
            unrefinableTetrahedra_.insert(tetId);
            continue;
        }

        // Demotion: if the circumcenter would encroach a segment, split that segment instead.
        const auto encroachingIds = curveSegmentManager.findEncroached(circumcenter, nodePositionMap);
        if (!encroachingIds.empty())
            return splitSegment(encroachingIds[0]);

        // Interior point: no geometryIds, matching insertVertexBowyerWatson's
        // convention for a non-boundary node.
        insertAndUpdate(circumcenter, {});
        return true;
    }

    return false;
}

bool RCDTRefiner::refineNonManifoldEdges()
{
    const auto& meshData = context_->getMeshData();
    const auto& curveSegmentManager = meshData.getCurveSegmentManager();
    const auto* geometry = context_->getGeometry();
    if (!geometry)
        return false;

    const auto nodePositionMap = buildNodePositionMap();
    const auto defects = restrictedTriangulation_->findNonManifoldEdges();

    for (const auto& defect : defects)
    {
        if (unrefinableNonManifoldEdges_.count(defect.edge))
            continue;

        const auto it1 = nodePositionMap.find(defect.edge.nodeIds[0]);
        const auto it2 = nodePositionMap.find(defect.edge.nodeIds[1]);
        if (it1 == nodePositionMap.end() || it2 == nodePositionMap.end())
        {
            unrefinableNonManifoldEdges_.insert(defect.edge);
            continue;
        }

        // Size floor, same reasoning as the other priorities.
        const double length = (it1->second - it2->second).norm();
        if (length <= minimumEdgeLength_)
        {
            unrefinableNonManifoldEdges_.insert(defect.edge);
            continue;
        }

        // If the defect's two endpoints are directly connected by a curve
        // segment, split that segment rather than projecting a point onto
        // one of the surfaces it touches. Every leak traced back during
        // OPE-170/171's investigation sat essentially exactly on a real CAD
        // edge (a crease between two surfaces) -- classifyFace's
        // candidate-surface disambiguation is hardest exactly there. A
        // generic surface projection lands *near* that crease but not
        // exactly on it, which doesn't resolve the defect so much as nudge
        // it elsewhere -- confirmed empirically: it was tried first and the
        // non-manifold-edge count grew instead of shrinking. Splitting the
        // segment keeps the new point exactly on the true curve, the same
        // arc-length-midpoint machinery priority 1 already uses.
        if (const auto segmentId = curveSegmentManager.findSegmentId(defect.edge.nodeIds[0], defect.edge.nodeIds[1]))
        {
            return splitSegment(*segmentId);
        }

        const Geometry3D::ISurface3D* surface = geometry->getSurface(defect.surfaceId);
        if (!surface)
        {
            unrefinableNonManifoldEdges_.insert(defect.edge);
            continue;
        }

        const Point3D midpoint = 0.5 * (it1->second + it2->second);
        const auto projectedOpt = surfaceProjector_.projectToSurface(midpoint, *surface);
        if (!projectedOpt)
        {
            unrefinableNonManifoldEdges_.insert(defect.edge);
            continue;
        }
        const Point3D& projected = *projectedOpt;

        // Proximity guard, same reasoning as the other priorities.
        bool tooClose = false;
        for (const auto& [nodeId, node] : meshData.getNodes())
        {
            if ((projected - node->getCoordinates()).norm() < minimumEdgeLength_)
            {
                tooClose = true;
                break;
            }
        }
        if (tooClose)
        {
            unrefinableNonManifoldEdges_.insert(defect.edge);
            continue;
        }

        // Demotion: if the insertion point would encroach a segment, split that segment instead.
        const auto encroachingIds = curveSegmentManager.findEncroached(projected, nodePositionMap);
        if (!encroachingIds.empty())
            return splitSegment(encroachingIds[0]);

        insertAndUpdate(projected, {defect.surfaceId});
        return true;
    }

    return false;
}

size_t RCDTRefiner::insertAndUpdate(const Point3D& point,
                                    const std::vector<std::string>& geometryIds)
{
    auto& operations = context_->getOperations();
    const auto& meshData = context_->getMeshData();
    const auto* geometry = context_->getGeometry();

    const auto conflictingTets = operations.getQueries().findConflictingTetrahedra(point);
    const auto interiorFaces = computeCavityInteriorFaces(conflictingTets);

    const size_t newNodeId = operations.insertVertexBowyerWatson(point, geometryIds);

    const MeshConnectivity postConnectivity(meshData);
    restrictedTriangulation_->updateAfterInsertion(
        interiorFaces, newNodeId, meshData, postConnectivity, *geometry);

    return newNodeId;
}

bool RCDTRefiner::splitSegment(size_t segmentId)
{
    const auto& meshData = context_->getMeshData();
    const auto* geometry = context_->getGeometry();
    if (!geometry)
        return false;

    const CurveSegment segment = meshData.getCurveSegmentManager().getSegment(segmentId);
    const Geometry3D::IEdge3D* edge = geometry->getEdge(segment.edgeId);
    if (!edge)
        return false;

    const Point3D splitPoint = computeSplitPoint(segment, *geometry);

    auto& operations = context_->getOperations();
    const auto conflictingTets = operations.getQueries().findConflictingTetrahedra(splitPoint);
    const auto interiorFaces = computeCavityInteriorFaces(conflictingTets);

    const size_t newNodeId = operations.insertVertexBowyerWatson(splitPoint, {segment.edgeId});

    const double tMid =
        edge->getParameterAtArcLengthFraction(segment.tStart, segment.tEnd, 0.5);
    context_->getMutator().splitCurveSegment(segmentId, newNodeId, tMid);

    restrictedTriangulation_->invalidateFacesWithEdge(segment.nodeId1, segment.nodeId2);

    const MeshConnectivity postConnectivity(meshData);
    restrictedTriangulation_->updateAfterInsertion(interiorFaces, newNodeId, meshData, postConnectivity, *geometry);

    unrefinableTriangles_.clear();
    unrefinableTetrahedra_.clear();
    unrefinableSegments_.clear();
    unrefinableNonManifoldEdges_.clear();
    return true;
}

double RCDTRefiner::resolveMinimumEdgeLength() const
{
    if (settings_.minimumEdgeLength)
        return *settings_.minimumEdgeLength;

    const auto& meshData = context_->getMeshData();
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

    // Median, not minimum: a periodic curve's discretization (e.g. a
    // cylinder's circular edges either side of the OCC seam) leaves one
    // short "remainder" segment wherever the curve length doesn't divide
    // evenly into whole angle steps starting from the seam vertex. A raw
    // minimum reliably picks up that artifact instead of the intended
    // spacing; the median is robust to the handful of short segments this
    // produces (see project memory: Linear ticket on removing OCC seams).
    std::sort(nearestPerNode.begin(), nearestPerNode.end());
    const double median = nearestPerNode[nearestPerNode.size() / 2];
    return median / AUTO_MINIMUM_EDGE_LENGTH_DIVISOR;
}

std::unordered_map<size_t, Point3D> RCDTRefiner::buildNodePositionMap() const
{
    std::unordered_map<size_t, Point3D> positionMap;
    for (const auto& [nodeId, node] : context_->getMeshData().getNodes())
        positionMap.emplace(nodeId, node->getCoordinates());
    return positionMap;
}

std::vector<FaceKey> RCDTRefiner::computeCavityInteriorFaces(
    const std::vector<size_t>& conflictingTets) const
{
    const auto& meshData = context_->getMeshData();

    std::unordered_map<FaceKey, size_t, FaceKeyHash> faceCount;
    for (const size_t tetId : conflictingTets)
    {
        const auto* element = meshData.getElement(tetId);
        const auto* tet = dynamic_cast<const TetrahedralElement*>(element);
        if (!tet)
            continue;

        for (const auto& faceArray : tet->getFaces())
            ++faceCount[FaceKey(faceArray)];
    }

    std::vector<FaceKey> interiorFaces;
    for (const auto& [face, count] : faceCount)
    {
        if (count == 2)
            interiorFaces.push_back(face);
    }

    return interiorFaces;
}

} // namespace Meshing
