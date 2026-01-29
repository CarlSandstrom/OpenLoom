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

        if (circle && GeometryUtilities2D::isPointInsideCircle(*circle, point))
        {
            conflicting.push_back(id);
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
        if (triangle->getHasNode(nodeId1) && triangle->getHasNode(nodeId2))
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

            if (segmentsIntersect(p1, p2, coordinate1, coordinate2))
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

bool MeshQueries2D::segmentsIntersect(const Point2D& a1, const Point2D& a2,
                                      const Point2D& b1, const Point2D& b2) const
{
    return GeometryUtilities2D::segmentsIntersect(a1, a2, b1, b2);
}

std::vector<ConstrainedSegment2D> MeshQueries2D::extractConstrainedEdges(
    const Topology2D::Topology2D& topology,
    const std::map<std::string, size_t>& cornerIdToPointIndexMap,
    const std::map<size_t, size_t>& pointIndexToNodeIdMap,
    const std::map<std::string, std::vector<size_t>>& edgeIdToPointIndicesMap) const
{
    std::vector<ConstrainedSegment2D> constrainedEdges;

    for (const auto& edgeId : topology.getAllEdgeIds())
    {
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

                constrainedEdges.push_back(ConstrainedSegment2D{startNodeId, endNodeId});

                spdlog::info("Edge {} segment {}: Node IDs ({}, {})", edgeId, i, startNodeId, endNodeId);
            }
        }
        else
        {
            const auto edgeTopology = topology.getEdge(edgeId);

            size_t startNodeId = pointIndexToNodeIdMap.at(
                cornerIdToPointIndexMap.at(edgeTopology.getStartCornerId()));
            size_t endNodeId = pointIndexToNodeIdMap.at(
                cornerIdToPointIndexMap.at(edgeTopology.getEndCornerId()));

            constrainedEdges.push_back(ConstrainedSegment2D{startNodeId, endNodeId});

            spdlog::info("Edge {}: Node IDs ({}, {})", edgeId, startNodeId, endNodeId);
        }
    }

    return constrainedEdges;
}

std::vector<ConstrainedSegment2D> MeshQueries2D::findEncroachedSegments(
    const std::vector<ConstrainedSegment2D>& constrainedSegments) const
{
    std::vector<ConstrainedSegment2D> encroached;

    ConstraintChecker2D checker(meshData_);

    for (const auto& segment : constrainedSegments)
    {
        for (const auto& [nodeId, node] : meshData_.getNodes())
        {
            if (nodeId == segment.nodeId1 || nodeId == segment.nodeId2)
                continue;

            Point2D point = node->getCoordinates();

            if (checker.isSegmentEncroached(segment, point))
            {
                encroached.push_back(segment);
                break;
            }
        }
    }

    return encroached;
}

std::vector<ConstrainedSegment2D> MeshQueries2D::findSegmentsEncroachedByPoint(
    const Point2D& point,
    const std::vector<ConstrainedSegment2D>& constrainedSegments) const
{
    std::vector<ConstrainedSegment2D> encroached;

    ConstraintChecker2D checker(meshData_);

    for (const auto& segment : constrainedSegments)
    {
        if (checker.isSegmentEncroached(segment, point))
        {
            encroached.push_back(segment);
        }
    }

    return encroached;
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

std::unordered_set<size_t> MeshQueries2D::classifyTrianglesInteriorExterior(
    const std::vector<ConstrainedSegment2D>& constrainedEdges) const
{
    std::unordered_set<size_t> insideTriangles;

    if (meshData_.getElements().empty())
    {
        spdlog::warn("classifyTrianglesInteriorExterior: No triangles in mesh");
        return insideTriangles;
    }

    if (constrainedEdges.empty())
    {
        spdlog::warn("classifyTrianglesInteriorExterior: No constraint edges, skipping classification");
        return insideTriangles;
    }

    // Helper: Check if an edge is a constraint edge
    auto isConstraintEdge = [&](size_t nodeId1, size_t nodeId2) -> bool
    {
        for (const auto& segment : constrainedEdges)
        {
            if ((segment.nodeId1 == nodeId1 && segment.nodeId2 == nodeId2) ||
                (segment.nodeId1 == nodeId2 && segment.nodeId2 == nodeId1))
            {
                return true;
            }
        }
        return false;
    };

    // Helper: Compute distance from point to line segment
    auto pointToSegmentDistance = [](const Point2D& p, const Point2D& a, const Point2D& b) -> double
    {
        Point2D ab = b - a;
        Point2D ap = p - a;

        double abLengthSq = ab.x() * ab.x() + ab.y() * ab.y();
        if (abLengthSq < 1e-12)
        {
            return std::sqrt(ap.x() * ap.x() + ap.y() * ap.y());
        }

        double t = (ap.x() * ab.x() + ap.y() * ab.y()) / abLengthSq;
        t = std::max(0.0, std::min(1.0, t));

        Point2D projection = a + Point2D(ab.x() * t, ab.y() * t);
        Point2D diff = p - projection;
        return std::sqrt(diff.x() * diff.x() + diff.y() * diff.y());
    };

    // Helper: Compute centroid of triangle
    auto computeCentroid = [&](const TriangleElement* triangle) -> Point2D
    {
        const auto& nodeIds = triangle->getNodeIdArray();
        Point2D p0 = meshData_.getNode(nodeIds[0])->getCoordinates();
        Point2D p1 = meshData_.getNode(nodeIds[1])->getCoordinates();
        Point2D p2 = meshData_.getNode(nodeIds[2])->getCoordinates();
        return Point2D((p0.x() + p1.x() + p2.x()) / 3.0,
                       (p0.y() + p1.y() + p2.y()) / 3.0);
    };

    // Step 1: Find seed triangle (farthest from all constraint edges)
    const TriangleElement* seedTriangle = nullptr;
    size_t seedTriangleId = SIZE_MAX;
    double maxMinDistance = -1.0;

    for (const auto& [elemId, element] : meshData_.getElements())
    {
        const auto* triangle = dynamic_cast<const TriangleElement*>(element.get());
        if (!triangle)
            continue;

        Point2D centroid = computeCentroid(triangle);
        double minDistToConstraint = std::numeric_limits<double>::max();

        for (const auto& segment : constrainedEdges)
        {
            const Node2D* node1 = meshData_.getNode(segment.nodeId1);
            const Node2D* node2 = meshData_.getNode(segment.nodeId2);

            if (!node1 || !node2)
                continue;

            Point2D p1 = node1->getCoordinates();
            Point2D p2 = node2->getCoordinates();
            double dist = pointToSegmentDistance(centroid, p1, p2);
            minDistToConstraint = std::min(minDistToConstraint, dist);
        }

        if (minDistToConstraint > maxMinDistance)
        {
            maxMinDistance = minDistToConstraint;
            seedTriangle = triangle;
            seedTriangleId = elemId;
        }
    }

    if (!seedTriangle || seedTriangleId == SIZE_MAX)
    {
        spdlog::error("classifyTrianglesInteriorExterior: Could not find seed triangle");
        return insideTriangles;
    }

    spdlog::info("classifyTrianglesInteriorExterior: Found seed triangle {} with min distance {} to constraints",
                 seedTriangleId, maxMinDistance);

    // Step 2: Build edge-to-triangles adjacency map
    using EdgeKey = std::pair<size_t, size_t>;
    struct EdgeKeyHash
    {
        std::size_t operator()(const EdgeKey& key) const
        {
            return std::hash<size_t>{}(key.first) ^ (std::hash<size_t>{}(key.second) << 1);
        }
    };
    std::unordered_map<EdgeKey, std::vector<size_t>, EdgeKeyHash> edgeToTriangles;

    for (const auto& [elemId, element] : meshData_.getElements())
    {
        const auto* triangle = dynamic_cast<const TriangleElement*>(element.get());
        if (!triangle)
            continue;

        for (size_t i = 0; i < 3; ++i)
        {
            auto edge = triangle->getEdge(i);
            EdgeKey key = makeEdgeKey(edge[0], edge[1]);
            edgeToTriangles[key].push_back(elemId);
        }
    }

    // Step 3: Perform BFS flood fill starting from seed triangle
    std::queue<size_t> queue;

    queue.push(seedTriangleId);
    insideTriangles.insert(seedTriangleId);

    size_t visitedCount = 0;
    while (!queue.empty())
    {
        size_t currentTriId = queue.front();
        queue.pop();
        visitedCount++;

        const IElement* elem = meshData_.getElement(currentTriId);
        if (!elem)
            continue;

        const auto* triangle = dynamic_cast<const TriangleElement*>(elem);
        if (!triangle)
            continue;

        // Check all three edges of the current triangle
        for (size_t i = 0; i < 3; ++i)
        {
            auto edge = triangle->getEdge(i);
            size_t node1 = edge[0];
            size_t node2 = edge[1];

            // Don't cross constraint edges
            if (isConstraintEdge(node1, node2))
            {
                continue;
            }

            // Find adjacent triangle through this edge
            EdgeKey edgeKey = makeEdgeKey(node1, node2);
            auto it = edgeToTriangles.find(edgeKey);
            if (it == edgeToTriangles.end())
            {
                continue;
            }

            const auto& adjacentTriangles = it->second;

            // Find the neighbor (the triangle that is not currentTriId)
            for (size_t neighborId : adjacentTriangles)
            {
                if (neighborId == currentTriId)
                    continue;

                // If we haven't visited this neighbor yet, add it to the flood fill
                if (insideTriangles.find(neighborId) == insideTriangles.end())
                {
                    insideTriangles.insert(neighborId);
                    queue.push(neighborId);
                }
            }
        }
    }

    spdlog::info("classifyTrianglesInteriorExterior: Flood fill visited {} triangles", visitedCount);
    spdlog::info("classifyTrianglesInteriorExterior: Marked {} triangles as inside", insideTriangles.size());

    return insideTriangles;
}

} // namespace Meshing
