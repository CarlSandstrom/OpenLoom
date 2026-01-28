#include "MeshQueries2D.h"
#include "ConstraintChecker2D.h"
#include "ElementGeometry2D.h"
#include "GeometryUtilities2D.h"
#include "Meshing/Data/2D/Node2D.h"
#include "Topology2D/Edge2D.h"
#include "Topology2D/Topology2D.h"
#include "spdlog/spdlog.h"
#include <algorithm>

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

} // namespace Meshing
