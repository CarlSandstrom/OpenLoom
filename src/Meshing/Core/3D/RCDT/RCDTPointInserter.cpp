#include "Meshing/Core/3D/RCDT/RCDTPointInserter.h"

#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/IEdge3D.h"
#include "Meshing/Connectivity/FaceKey.h"
#include "Meshing/Core/3D/General/MeshOperations3D.h"
#include "Meshing/Core/3D/General/MeshQueries3D.h"
#include "Meshing/Core/3D/General/MeshingContext3D.h"
#include "Meshing/Core/3D/RCDT/CurveSegmentOperations.h"
#include "Meshing/Core/3D/RCDT/RestrictedTriangulation.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Meshing/Data/3D/Node3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"
#include "Meshing/Data/Base/MeshConnectivity.h"
#include "Meshing/Data/CurveSegmentManager.h"

namespace Meshing
{

namespace
{

/// Builds a node-ID → position lookup from the current mesh.
std::unordered_map<size_t, Point3D> buildNodePositionMap(const MeshData3D& meshData)
{
    std::unordered_map<size_t, Point3D> nodePositions;
    for (const auto& [nodeId, node] : meshData.getNodes())
        nodePositions.emplace(nodeId, node->getCoordinates());
    return nodePositions;
}

/// True if point lies closer than distance to any node of the mesh.
bool isWithinDistanceOfAnyNode(const MeshData3D& meshData, const Point3D& point, double distance)
{
    for (const auto& [nodeId, node] : meshData.getNodes())
    {
        if ((point - node->getCoordinates()).norm() < distance)
            return true;
    }
    return false;
}

/// True if point lies strictly inside the protecting ball of a node with a
/// positive weight (see Node3D::getWeight() and CurveProtectionScheme,
/// OPE-176). An insertion there would erode the crease protection the
/// ball exists for. A ball cannot be split the way an encroached segment
/// can, so the candidate is marked unrefinable instead.
bool encroachesProtectingBall(const MeshData3D& meshData, const Point3D& point)
{
    for (const auto& [nodeId, node] : meshData.getNodes())
    {
        const double weight = node->getWeight();
        if (weight <= 0.0)
            continue;
        if ((point - node->getCoordinates()).squaredNorm() < weight)
            return true;
    }
    return false;
}

/// Returns the FaceKeys of faces shared by exactly two of conflictingTetrahedra:
/// the cavity's interior faces, as the conflict set stands before insertion.
std::vector<FaceKey> computeCavityInteriorFaces(const MeshData3D& meshData,
                                                const std::vector<size_t>& conflictingTetrahedra)
{
    std::unordered_map<FaceKey, size_t, FaceKeyHash> conflictingTetrahedraPerFace;
    for (const size_t tetrahedronId : conflictingTetrahedra)
    {
        const auto* element = meshData.getElement(tetrahedronId);
        const auto* tetrahedron = dynamic_cast<const TetrahedralElement*>(element);
        if (!tetrahedron)
            continue;

        for (const auto& faceArray : tetrahedron->getFaces())
            ++conflictingTetrahedraPerFace[FaceKey(faceArray)];
    }

    std::vector<FaceKey> interiorFaces;
    for (const auto& [face, count] : conflictingTetrahedraPerFace)
    {
        if (count == 2)
            interiorFaces.push_back(face);
    }

    return interiorFaces;
}

} // namespace

RCDTPointInserter::RCDTPointInserter(MeshingContext3D& context,
                                     RestrictedTriangulation& restrictedTriangulation,
                                     double minimumEdgeLength) :
    context_(&context),
    restrictedTriangulation_(&restrictedTriangulation),
    minimumEdgeLength_(minimumEdgeLength)
{
}

void RCDTPointInserter::seedEncroachedSegments()
{
    const auto& meshData = context_->getMeshData();
    const auto& curveSegmentManager = meshData.getCurveSegmentManager();
    const auto nodePositionMap = buildNodePositionMap(meshData);
    for (const auto& [nodeId, node] : meshData.getNodes())
    {
        for (const size_t segmentId :
             curveSegmentManager.findEncroached(node->getCoordinates(), nodePositionMap, nodeId))
        {
            encroachedSegments_.insert(segmentId);
        }
    }
}

void RCDTPointInserter::beginStep()
{
    cachedNodePositionMap_.reset();
}

bool RCDTPointInserter::splitEncroachedSegment()
{
    const auto& meshData = context_->getMeshData();
    const auto& curveSegmentManager = meshData.getCurveSegmentManager();

    for (const size_t segmentId : encroachedSegments_)
    {
        if (unrefinableSegments_.count(segmentId))
            continue;

        // Size floor: a segment at or below minimumEdgeLength_ is left
        // encroached rather than bisected forever (see unrefinableSegments_).
        const CurveSegment segment = curveSegmentManager.getSegment(segmentId);
        const auto* node1 = meshData.getNode(segment.nodeId1);
        const auto* node2 = meshData.getNode(segment.nodeId2);
        const double length = (node1 && node2) ? (node1->getCoordinates() - node2->getCoordinates()).norm() : 0.0;
        if (length <= minimumEdgeLength_)
        {
            unrefinableSegments_.insert(segmentId);
            continue;
        }

        // trySplitSegment() already marks segmentId unrefinable when it
        // declines (ball-encroached or otherwise) -- try the next
        // encroached segment instead of giving up on this refinement step
        // entirely.
        if (trySplitSegment(segmentId))
            return true;
    }

    return false;
}

bool RCDTPointInserter::tryInsert(const Point3D& point, const std::vector<std::string>& geometryIds)
{
    const auto& meshData = context_->getMeshData();

    // Proximity guard: reject a point within minimumEdgeLength_ of any
    // existing vertex. A candidate's own size floor only sees it before
    // insertion, so it cannot catch a point landing on or next to a vertex,
    // including one of an unrelated candidate. Such a near-duplicate corrupts
    // the mesh locally and keeps generating new bad elements around it.
    if (isWithinDistanceOfAnyNode(meshData, point, minimumEdgeLength_))
        return false;

    // Demotion: if the point would encroach a segment, split that segment
    // instead. If trySplitSegment() declines, the candidate has no other way
    // to be refined, so the caller marks it unrefinable rather than retrying.
    const auto encroachedSegmentIds = meshData.getCurveSegmentManager().findEncroached(point, getNodePositionMap());
    if (!encroachedSegmentIds.empty())
        return trySplitSegment(encroachedSegmentIds[0]);

    // Never insert inside an existing protecting ball (OPE-176) -- see
    // encroachesProtectingBall()'s doc.
    if (encroachesProtectingBall(meshData, point))
        return false;

    const Insertion insertion = insertPoint(point, geometryIds);
    finishInsertion(insertion, point);
    return true;
}

bool RCDTPointInserter::trySplitSegment(size_t segmentId)
{
    const auto& meshData = context_->getMeshData();
    const auto* geometry = context_->getGeometry();
    if (!geometry)
    {
        unrefinableSegments_.insert(segmentId);
        return false;
    }

    // Copied rather than referenced: splitSegment() still reads it after
    // splitCurveSegment() has destroyed the original.
    const CurveSegment segment = meshData.getCurveSegmentManager().getSegment(segmentId);
    const Geometry3D::IEdge3D* edge = geometry->getEdge(segment.edgeId);
    if (!edge)
    {
        unrefinableSegments_.insert(segmentId);
        return false;
    }

    const Point3D splitPoint = CurveSegmentOperations::computeSplitPoint(segment, *geometry);
    if (encroachesProtectingBall(meshData, splitPoint))
    {
        unrefinableSegments_.insert(segmentId);
        return false;
    }

    splitSegment(segmentId, segment, *edge, splitPoint);
    return true;
}

const std::unordered_map<size_t, Point3D>& RCDTPointInserter::getNodePositionMap()
{
    if (!cachedNodePositionMap_)
        cachedNodePositionMap_ = buildNodePositionMap(context_->getMeshData());
    return *cachedNodePositionMap_;
}

RCDTPointInserter::Insertion RCDTPointInserter::insertPoint(const Point3D& point,
                                                            const std::vector<std::string>& geometryIds)
{
    auto& operations = context_->getOperations();
    const auto& meshData = context_->getMeshData();

    auto conflictingTetrahedra = operations.getQueries().findConflictingTetrahedra(point);

    Insertion insertion;
    insertion.cavityInteriorFaces = computeCavityInteriorFaces(meshData, conflictingTetrahedra);
    insertion.newNodeId = operations.insertVertexBowyerWatson(point, std::move(conflictingTetrahedra), geometryIds);
    return insertion;
}

void RCDTPointInserter::finishInsertion(const Insertion& insertion, const Point3D& position)
{
    const auto& meshData = context_->getMeshData();

    const MeshConnectivity connectivityAfterInsertion(meshData);
    restrictedTriangulation_->updateAfterInsertion(insertion.cavityInteriorFaces,
                                                   insertion.newNodeId,
                                                   meshData,
                                                   connectivityAfterInsertion,
                                                   *context_->getGeometry());

    markSegmentsEncroachedBy(insertion.newNodeId, position);
}

void RCDTPointInserter::splitSegment(size_t segmentId,
                                     const CurveSegment& segment,
                                     const Geometry3D::IEdge3D& edge,
                                     const Point3D& splitPoint)
{
    const Insertion insertion = insertPoint(splitPoint, {segment.edgeId});

    // The curve segments are updated between the insertion and
    // finishInsertion(): classifyFace() reads them, and invalidating faces
    // after that reclassification would drop what it had just rebuilt.
    const double midpointParameter = edge.getParameterAtArcLengthFraction(segment.tStart, segment.tEnd, 0.5);
    const auto [segmentId1, segmentId2] =
        context_->getMutator().splitCurveSegment(segmentId, insertion.newNodeId, midpointParameter);

    // The original segment is gone; the two it split into inherit its
    // encroachment status only insofar as they're recomputed below -- an
    // existing, unrelated node can already sit inside one of the (smaller)
    // new diametral spheres even though it didn't encroach the original.
    encroachedSegments_.erase(segmentId);

    restrictedTriangulation_->invalidateFacesWithEdge(segment.nodeId1, segment.nodeId2);

    finishInsertion(insertion, splitPoint);
    markSegmentIfEncroached(segmentId1);
    markSegmentIfEncroached(segmentId2);
}

void RCDTPointInserter::markSegmentsEncroachedBy(size_t newNodeId, const Point3D& position)
{
    const auto& meshData = context_->getMeshData();
    const auto& nodePositionMap = getNodePositionMap();
    for (const size_t segmentId :
         meshData.getCurveSegmentManager().findEncroached(position, nodePositionMap, newNodeId))
    {
        encroachedSegments_.insert(segmentId);
    }
}

void RCDTPointInserter::markSegmentIfEncroached(size_t segmentId)
{
    const auto& meshData = context_->getMeshData();
    const CurveSegment& segment = meshData.getCurveSegmentManager().getSegment(segmentId);
    const auto* node1 = meshData.getNode(segment.nodeId1);
    const auto* node2 = meshData.getNode(segment.nodeId2);
    if (!node1 || !node2)
        return;

    const Point3D center = (node1->getCoordinates() + node2->getCoordinates()) * 0.5;
    const double radiusSquared = (node2->getCoordinates() - node1->getCoordinates()).squaredNorm() * 0.25;

    for (const auto& [nodeId, node] : meshData.getNodes())
    {
        if (nodeId == segment.nodeId1 || nodeId == segment.nodeId2)
            continue;
        if ((node->getCoordinates() - center).squaredNorm() < radiusSquared)
        {
            encroachedSegments_.insert(segmentId);
            return;
        }
    }
}

} // namespace Meshing
