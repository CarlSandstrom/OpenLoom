#include "MeshQueries2D.h"
#include "ConstraintChecker2D.h"
#include "ElementGeometry2D.h"
#include "GeometryUtilities2D.h"
#include "Meshing/Data/2D/Node2D.h"
#include "Topology2D/Edge2D.h"
#include "Topology2D/Topology2D.h"
#include "spdlog/spdlog.h"
#include <algorithm>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace Meshing
{

MeshQueries2D::MeshQueries2D(const MeshData2D& meshData) :
    meshData_(meshData)
{
}

std::vector<size_t> MeshQueries2D::findConflictingTriangles(const Point2D& point) const
{
    std::vector<size_t> conflicting;

    ElementGeometry2D geometry(meshData_);

    for (const auto& [id, element] : meshData_.getElements())
    {
        const auto* triangle = dynamic_cast<const TriangleElement*>(element.get());
        if (triangle == nullptr)
        {
            SPDLOG_WARN("MeshQueries2D: Skipping non-triangle element {}", id);
            continue;
        }

        auto circle = geometry.computeCircumcircle(*triangle);
        bool inConflict = circle && GeometryUtilities2D::isPointInsideCircle(*circle, point);
        if (!inConflict && !circle)
        {
            // Degenerate triangle: circumcircle is undefined (nearly collinear vertices).
            // Fall back to direct containment — a point on the triangle boundary
            // (e.g. the midpoint of a segment being split) must still be included.
            const auto& ids = triangle->getNodeIdArray();
            const Point2D& v0 = meshData_.getNode(ids[0])->getCoordinates();
            const Point2D& v1 = meshData_.getNode(ids[1])->getCoordinates();
            const Point2D& v2 = meshData_.getNode(ids[2])->getCoordinates();
            inConflict = GeometryUtilities2D::isPointInsideOrOnTriangle(point, v0, v1, v2);
        }
        if (inConflict)
        {
            // Visibility check per constrained Delaunay definition (Shewchuk, p.2):
            // A triangle is in the conflict set only if the insertion point is visible
            // from the triangle's interior, i.e. the line from point to centroid does
            // not properly cross any constrained segment.
            Point2D centroid = geometry.computeCentroid(*triangle);

            bool visible = true;
            const auto& nodeIds = triangle->getNodeIdArray();
            for (const auto& segment : meshData_.getConstrainedSegments())
            {
                // Skip segments that share a node with this triangle
                if (segment.nodeId1 == nodeIds[0] || segment.nodeId1 == nodeIds[1] ||
                    segment.nodeId1 == nodeIds[2] || segment.nodeId2 == nodeIds[0] ||
                    segment.nodeId2 == nodeIds[1] || segment.nodeId2 == nodeIds[2])
                {
                    continue;
                }

                const Point2D& s1 = meshData_.getNode(segment.nodeId1)->getCoordinates();
                const Point2D& s2 = meshData_.getNode(segment.nodeId2)->getCoordinates();

                // If the insertion point lies on this constraint segment, the line from
                // point to centroid starts on the segment — that is not a proper crossing
                // and cannot block visibility (e.g. when splitting a seam segment at its midpoint).
                if (GeometryUtilities2D::computeOrientationSign(s1, s2, point) == 0 &&
                    point.x() >= std::min(s1.x(), s2.x()) - 1e-10 &&
                    point.x() <= std::max(s1.x(), s2.x()) + 1e-10 &&
                    point.y() >= std::min(s1.y(), s2.y()) - 1e-10 &&
                    point.y() <= std::max(s1.y(), s2.y()) + 1e-10)
                {
                    continue;
                }

                if (GeometryUtilities2D::segmentsIntersect(point, centroid, s1, s2))
                {
                    visible = false;
                    break;
                }
            }

            if (visible)
            {
                conflicting.push_back(id);
            }
        }
    }

    // Star-shapedness verification: remove cavity triangles whose boundary edges
    // would produce inverted triangles when connected to the insertion point
    // (guards against degenerate cases near constraint boundaries).
    bool changed = true;
    while (changed)
    {
        changed = false;

        std::unordered_map<EdgeKey, size_t, EdgeKeyHash> edgeCount;
        std::unordered_map<EdgeKey, std::array<size_t, 2>, EdgeKeyHash> edgeOriginal;
        std::unordered_map<EdgeKey, size_t, EdgeKeyHash> edgeOwner;

        for (size_t id : conflicting)
        {
            const auto* tri = dynamic_cast<const TriangleElement*>(meshData_.getElement(id));
            for (size_t i = 0; i < 3; ++i)
            {
                auto edge = tri->getEdge(i);
                EdgeKey key = makeEdgeKey(edge[0], edge[1]);
                edgeCount[key]++;
                edgeOriginal[key] = edge;
                edgeOwner[key] = id;
            }
        }

        for (const auto& [key, count] : edgeCount)
        {
            if (count != 1)
                continue;

            const auto& edge = edgeOriginal[key];
            const Point2D& p0 = meshData_.getNode(edge[0])->getCoordinates();
            const Point2D& p1 = meshData_.getNode(edge[1])->getCoordinates();
            double area = GeometryUtilities2D::computeSignedArea(point, p0, p1);

            if (area < -1e-10)
            {
                std::erase(conflicting, edgeOwner[key]);
                changed = true;
                break;
            }
        }
    }

    return conflicting;
}

std::vector<std::array<size_t, 2>> MeshQueries2D::findCavityBoundary(const std::vector<size_t>& conflictingIndices) const
{
    // Count how many times each edge appears
    std::map<std::pair<size_t, size_t>, int> edgeCount;
    std::map<std::pair<size_t, size_t>, std::array<size_t, 2>> edgeLookup;

    for (size_t idx : conflictingIndices)
    {
        auto tri = dynamic_cast<const TriangleElement&>(*meshData_.getElement(idx));
        for (size_t i = 0; i < 3; ++i)
        {
            auto edge = tri.getEdge(i);
            auto key = makeEdgeKey(edge[0], edge[1]);
            edgeCount[key]++;
            edgeLookup[key] = edge; // Store original order
        }
    }

    // Boundary edges appear exactly once
    std::vector<std::array<size_t, 2>> boundary;
    for (const auto& [key, count] : edgeCount)
    {
        if (count == 1)
        {
            boundary.push_back(edgeLookup.at(key));
        }
    }

    return boundary;
}

std::vector<size_t> MeshQueries2D::findIntersectingTriangles(size_t nodeId1, size_t nodeId2) const
{
    std::vector<size_t> result;

    const auto& p1 = meshData_.getNode(nodeId1)->getCoordinates();
    const auto& p2 = meshData_.getNode(nodeId2)->getCoordinates();

    for (auto& idAndTriangle : meshData_.getElements())
    {
        auto triangle = dynamic_cast<const TriangleElement*>(idAndTriangle.second.get());
        if (!triangle)
        {
            spdlog::warn("MeshQueries2D: Skipping non-triangle element {}", idAndTriangle.first);
            continue; // Skip non-triangle elements
        }

        // Skip triangles that already contain both nodes
        if (triangle->hasNode(nodeId1) && triangle->hasNode(nodeId2))
        {
            continue;
        }

        // Check if constraint edge intersects any edge of this triangle
        bool intersects = false;
        for (size_t edgeIndex = 0; edgeIndex < 3; ++edgeIndex)
        {
            auto edge = triangle->getEdge(edgeIndex);

            // Skip edges that share a node with the constraint edge
            if (edge[0] == nodeId1 || edge[0] == nodeId2 ||
                edge[1] == nodeId1 || edge[1] == nodeId2)
            {
                continue;
            }

            const Point2D& coordinate1 = meshData_.getNode(edge[0])->getCoordinates();
            const Point2D& coordinate2 = meshData_.getNode(edge[1])->getCoordinates();

            if (GeometryUtilities2D::segmentsIntersect(p1, p2, coordinate1, coordinate2))
            {
                intersects = true;
                break;
            }
        }

        if (intersects)
        {
            result.push_back(idAndTriangle.first);
        }
    }

    return result;
}

std::vector<ConstrainedSegment2D> MeshQueries2D::extractConstrainedEdges(const Topology2D::Topology2D& topology,
                                                                         const std::map<std::string, size_t>& cornerIdToPointIndexMap,
                                                                         const std::map<size_t, size_t>& pointIndexToNodeIdMap,
                                                                         const std::map<std::string, std::vector<size_t>>& edgeIdToPointIndicesMap) const
{
    std::vector<ConstrainedSegment2D> constrainedEdges;

    // Build set of boundary edge IDs (outer loop + hole loops)
    std::unordered_set<std::string> boundaryEdgeIds;
    for (const auto& edgeId : topology.getOuterEdgeLoop())
    {
        boundaryEdgeIds.insert(edgeId);
    }
    for (const auto& holeLoop : topology.getHoleEdgeLoops())
    {
        for (const auto& edgeId : holeLoop)
        {
            boundaryEdgeIds.insert(edgeId);
        }
    }

    for (const auto& edgeId : topology.getAllEdgeIds())
    {
        EdgeRole role = boundaryEdgeIds.count(edgeId) ? EdgeRole::BOUNDARY : EdgeRole::INTERIOR;

        auto edgePointsIt = edgeIdToPointIndicesMap.find(edgeId);

        if (edgePointsIt != edgeIdToPointIndicesMap.end() && edgePointsIt->second.size() >= 2)
        {
            const auto& pointIndices = edgePointsIt->second;

            for (size_t i = 0; i < pointIndices.size() - 1; ++i)
            {
                size_t startPointIdx = pointIndices[i];
                size_t endPointIdx = pointIndices[i + 1];

                size_t startNodeId = pointIndexToNodeIdMap.at(startPointIdx);
                size_t endNodeId = pointIndexToNodeIdMap.at(endPointIdx);

                constrainedEdges.push_back(ConstrainedSegment2D{startNodeId, endNodeId, role});

                spdlog::debug("Edge {} segment {}: Node IDs ({}, {}), role: {}",
                              edgeId, i, startNodeId, endNodeId,
                              role == EdgeRole::BOUNDARY ? "boundary" : "interior");
            }
        }
        else
        {
            const auto edgeTopology = topology.getEdge(edgeId);

            size_t startNodeId = pointIndexToNodeIdMap.at(
                cornerIdToPointIndexMap.at(edgeTopology.getStartCornerId()));
            size_t endNodeId = pointIndexToNodeIdMap.at(
                cornerIdToPointIndexMap.at(edgeTopology.getEndCornerId()));

            constrainedEdges.push_back(ConstrainedSegment2D{startNodeId, endNodeId, role});

            spdlog::debug("Edge {}: Node IDs ({}, {}), role: {}",
                          edgeId, startNodeId, endNodeId,
                          role == EdgeRole::BOUNDARY ? "boundary" : "interior");
        }
    }

    return constrainedEdges;
}

std::vector<ConstrainedSegment2D> MeshQueries2D::findEncroachedSegments() const
{
    std::vector<ConstrainedSegment2D> encroached;

    ConstraintChecker2D checker(meshData_);

    for (const auto& segment : meshData_.getConstrainedSegments())
    {
        for (const auto& [nodeId, node] : meshData_.getNodes())
        {
            if (nodeId == segment.nodeId1 || nodeId == segment.nodeId2)
                continue;

            const Point2D& point = node->getCoordinates();

            if (checker.isSegmentEncroached(segment, point))
            {
                // Per Ruppert: only count encroachment if the vertex is visible
                // from the subsegment interior (visibility obstructed by other segments)
                if (isPointVisibleFromSegment(point, segment))
                {
                    encroached.push_back(segment);
                    break;
                }
            }
        }
    }

    return encroached;
}

std::vector<ConstrainedSegment2D> MeshQueries2D::findSegmentsEncroachedByPoint(
    const Point2D& point) const
{
    std::vector<ConstrainedSegment2D> encroached;

    ConstraintChecker2D checker(meshData_);

    for (const auto& segment : meshData_.getConstrainedSegments())
    {
        if (checker.isSegmentEncroached(segment, point))
        {
            // Per Ruppert: only count encroachment if the point is visible
            // from the subsegment interior
            if (isPointVisibleFromSegment(point, segment))
            {
                encroached.push_back(segment);
            }
        }
    }

    return encroached;
}

bool MeshQueries2D::isPointVisibleFromSegment(const Point2D& point, const ConstrainedSegment2D& segment) const
{
    // Compute the midpoint of the subsegment
    const Point2D& s1 = meshData_.getNode(segment.nodeId1)->getCoordinates();
    const Point2D& s2 = meshData_.getNode(segment.nodeId2)->getCoordinates();
    Point2D midpoint = (s1 + s2) * 0.5;

    // Check if the line from point to midpoint properly crosses any other
    // constrained segment. "Properly" means the segments cross in their interiors,
    // not just touch at endpoints.
    for (const auto& other : meshData_.getConstrainedSegments())
    {
        // Skip the segment itself
        if (other.nodeId1 == segment.nodeId1 && other.nodeId2 == segment.nodeId2)
            continue;
        if (other.nodeId1 == segment.nodeId2 && other.nodeId2 == segment.nodeId1)
            continue;

        const Point2D& o1 = meshData_.getNode(other.nodeId1)->getCoordinates();
        const Point2D& o2 = meshData_.getNode(other.nodeId2)->getCoordinates();

        // Check for proper intersection (not at shared endpoints).
        // Segments sharing an endpoint with the query segment or touching at the
        // point itself should not block visibility.
        int d1 = GeometryUtilities2D::computeOrientationSign(point, midpoint, o1);
        int d2 = GeometryUtilities2D::computeOrientationSign(point, midpoint, o2);
        int d3 = GeometryUtilities2D::computeOrientationSign(o1, o2, point);
        int d4 = GeometryUtilities2D::computeOrientationSign(o1, o2, midpoint);

        // Proper crossing: both endpoints of each segment are strictly on opposite
        // sides of the other segment
        if (d1 != 0 && d2 != 0 && d1 != d2 && d3 != 0 && d4 != 0 && d3 != d4)
        {
            return false; // Blocked by this segment
        }
    }

    return true; // No segment blocks visibility
}

std::optional<std::string> MeshQueries2D::findCommonGeometryId(size_t nodeId1, size_t nodeId2) const
{
    const Node2D* node1 = meshData_.getNode(nodeId1);
    const Node2D* node2 = meshData_.getNode(nodeId2);

    if (node1 == nullptr || node2 == nullptr)
    {
        return std::nullopt;
    }

    const auto& geometryIds1 = node1->getGeometryIds();
    const auto& geometryIds2 = node2->getGeometryIds();

    for (const auto& id1 : geometryIds1)
    {
        for (const auto& id2 : geometryIds2)
        {
            if (id1 == id2)
            {
                return id1;
            }
        }
    }

    return std::nullopt;
}

std::vector<size_t> MeshQueries2D::findTrianglesAdjacentToEdge(size_t nodeId1, size_t nodeId2) const
{
    std::vector<size_t> adjacent;
    auto edgeKey = makeEdgeKey(nodeId1, nodeId2);

    for (const auto& [elemId, element] : meshData_.getElements())
    {
        const auto* triangle = dynamic_cast<const TriangleElement*>(element.get());
        if (!triangle)
            continue;

        for (size_t i = 0; i < 3; ++i)
        {
            auto edge = triangle->getEdge(i);
            if (makeEdgeKey(edge[0], edge[1]) == edgeKey)
            {
                adjacent.push_back(elemId);
                break;
            }
        }
    }

    return adjacent;
}

bool MeshQueries2D::isPointInsideDomain(const Point2D& point) const
{
    int crossings = 0;
    double px = point.x();
    double py = point.y();

    for (const auto& segment : meshData_.getConstrainedSegments())
    {
        if (segment.role != EdgeRole::BOUNDARY)
            continue;

        const Node2D* n1 = meshData_.getNode(segment.nodeId1);
        const Node2D* n2 = meshData_.getNode(segment.nodeId2);
        if (!n1 || !n2)
            continue;

        Point2D a = n1->getCoordinates();
        Point2D b = n2->getCoordinates();
        double ay = a.y();
        double by = b.y();

        // Check if the segment straddles the ray's Y coordinate
        if ((ay <= py && by > py) || (by <= py && ay > py))
        {
            // Compute X coordinate of intersection with the horizontal ray
            double t = (py - ay) / (by - ay);
            double ix = a.x() + t * (b.x() - a.x());

            if (px < ix)
            {
                crossings++;
            }
        }
    }

    return (crossings % 2) == 1;
}

std::unordered_set<size_t> MeshQueries2D::classifyTrianglesInteriorExterior() const
{
    std::unordered_set<size_t> insideTriangles;

    if (meshData_.getElements().empty())
    {
        spdlog::warn("classifyTrianglesInteriorExterior: No triangles in mesh");
        return insideTriangles;
    }

    const auto& constrainedEdges = meshData_.getConstrainedSegments();

    if (constrainedEdges.empty())
    {
        spdlog::warn("classifyTrianglesInteriorExterior: No constraint edges, skipping classification");
        return insideTriangles;
    }

    // Step 1: Find an interior seed triangle using ray casting
    ElementGeometry2D geometry(meshData_);
    size_t seedTriangleId = SIZE_MAX;

    for (const auto& [elemId, element] : meshData_.getElements())
    {
        const auto* triangle = dynamic_cast<const TriangleElement*>(element.get());
        if (!triangle)
            continue;

        Point2D centroid = geometry.computeCentroid(*triangle);
        if (isPointInsideDomain(centroid))
        {
            seedTriangleId = elemId;
            break;
        }
    }

    if (seedTriangleId == SIZE_MAX)
    {
        spdlog::error("classifyTrianglesInteriorExterior: Ray casting found no interior triangle");
        return insideTriangles;
    }

    spdlog::debug("classifyTrianglesInteriorExterior: Ray casting found interior seed triangle {}", seedTriangleId);

    // Step 2: Build edge-to-triangles adjacency map
    EdgeToTrianglesMap edgeToTriangles = buildEdgeToTrianglesMap();

    // Step 3: Flood fill from the interior seed, stopping at boundary constraints
    std::queue<size_t> queue;
    queue.push(seedTriangleId);
    insideTriangles.insert(seedTriangleId);

    while (!queue.empty())
    {
        size_t currentTriId = queue.front();
        queue.pop();

        const IElement* elem = meshData_.getElement(currentTriId);
        if (!elem)
            continue;

        const auto* triangle = dynamic_cast<const TriangleElement*>(elem);
        if (!triangle)
            continue;

        for (size_t i = 0; i < 3; ++i)
        {
            auto edge = triangle->getEdge(i);
            if (isBoundaryConstraintEdge(edge))
                continue;

            EdgeKey edgeKey = makeEdgeKey(edge[0], edge[1]);
            auto it = edgeToTriangles.find(edgeKey);
            if (it == edgeToTriangles.end())
                continue;

            for (size_t neighborId : it->second)
            {
                if (neighborId != currentTriId &&
                    !insideTriangles.contains(neighborId))
                {
                    insideTriangles.insert(neighborId);
                    queue.push(neighborId);
                }
            }
        }
    }

    spdlog::debug("classifyTrianglesInteriorExterior: Flood fill found {} interior triangles", insideTriangles.size());

    return insideTriangles;
}

MeshQueries2D::EdgeToTrianglesMap MeshQueries2D::buildEdgeToTrianglesMap() const
{
    EdgeToTrianglesMap edgeToTriangles;

    for (const auto& [elemId, element] : meshData_.getElements())
    {
        const auto* triangle = dynamic_cast<const TriangleElement*>(element.get());
        if (!triangle)
        {
            continue;
        }

        for (size_t i = 0; i < 3; ++i)
        {
            auto edge = triangle->getEdge(i);
            EdgeKey key = makeEdgeKey(edge[0], edge[1]);
            edgeToTriangles[key].push_back(elemId);
        }
    }

    return edgeToTriangles;
}

bool MeshQueries2D::isBoundaryConstraintEdge(const std::array<size_t, 2>& edgeId) const
{
    const auto& constrainedEdges = meshData_.getConstrainedSegments();
    for (const auto& segment : constrainedEdges)
    {
        if (segment.role != EdgeRole::BOUNDARY)
        {
            continue;
        }

        if ((segment.nodeId1 == edgeId[0] && segment.nodeId2 == edgeId[1]) ||
            (segment.nodeId1 == edgeId[1] && segment.nodeId2 == edgeId[0]))
        {
            return true;
        }
    }

    return false;
}

} // namespace Meshing
