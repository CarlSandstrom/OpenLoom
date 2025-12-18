#include "MeshOperations2D.h"

#include "Computer.h"
#include "Meshing/Data/MeshMutator2D.h"

#include <algorithm>
#include <cmath>
#include <map>

#include "spdlog/spdlog.h"

namespace Meshing
{

MeshOperations2D::MeshOperations2D(MeshData2D& meshData) :
    meshData_(meshData),
    mutator_(std::make_unique<MeshMutator2D>(meshData))
{
}

void MeshOperations2D::insertVertexBowyerWatson(size_t nodeId,
                                                const std::unordered_map<size_t, Point2D>& nodeCoords,
                                                std::vector<TriangleElement>& activeTriangles) const
{
    const Point2D& point = nodeCoords.at(nodeId);

    // Find conflicting triangles
    std::vector<size_t> conflicting = findConflictingTriangles(point, nodeCoords, activeTriangles);

    if (conflicting.empty())
    {
        SPDLOG_WARN("MeshOperations2D: No conflicting triangles found for point ({}, {})",
                    point.x(), point.y());
        return;
    }

    // Find cavity boundary
    std::vector<std::array<size_t, 2>> boundary = findCavityBoundary(conflicting, activeTriangles);

    // Remove conflicting triangles
    std::vector<TriangleElement> newActiveTriangles;
    newActiveTriangles.reserve(activeTriangles.size());
    for (size_t i = 0; i < activeTriangles.size(); ++i)
    {
        if (std::find(conflicting.begin(), conflicting.end(), i) == conflicting.end())
        {
            newActiveTriangles.push_back(activeTriangles[i]);
        }
    }
    activeTriangles = std::move(newActiveTriangles);

    // Retriangulate cavity
    retriangulate(nodeId, boundary, activeTriangles);
}

std::vector<size_t> MeshOperations2D::findConflictingTriangles(
    const Point2D& point,
    const std::unordered_map<size_t, Point2D>& nodeCoords,
    const std::vector<TriangleElement>& activeTriangles) const
{
    std::vector<size_t> conflicting;

    for (size_t i = 0; i < activeTriangles.size(); ++i)
    {
        const auto& tri = activeTriangles[i];
        auto circle = Computer::computeCircumcircle(tri, nodeCoords);

        if (circle && Computer::isPointInsideCircumcircle(*circle, point))
        {
            conflicting.push_back(i);
        }
    }

    return conflicting;
}

std::vector<std::array<size_t, 2>> MeshOperations2D::findCavityBoundary(
    const std::vector<size_t>& conflictingIndices,
    const std::vector<TriangleElement>& activeTriangles) const
{
    // Count how many times each edge appears
    std::map<std::pair<size_t, size_t>, int> edgeCount;
    std::map<std::pair<size_t, size_t>, std::array<size_t, 2>> edgeLookup;

    for (size_t idx : conflictingIndices)
    {
        const auto& tri = activeTriangles[idx];
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

std::vector<size_t> MeshOperations2D::findIntersectingTriangles(
    size_t nodeId1,
    size_t nodeId2,
    const std::unordered_map<size_t, Point2D>& nodeCoords,
    const std::vector<TriangleElement>& activeTriangles) const
{
    std::vector<size_t> result;

    const Point2D& p1 = nodeCoords.at(nodeId1);
    const Point2D& p2 = nodeCoords.at(nodeId2);

    for (size_t i = 0; i < activeTriangles.size(); ++i)
    {
        const auto& tri = activeTriangles[i];

        // Skip triangles that already contain both nodes
        if (tri.getHasNode(nodeId1) && tri.getHasNode(nodeId2))
        {
            continue;
        }

        // Check if constraint edge intersects any edge of this triangle
        bool intersects = false;
        for (size_t edgeIdx = 0; edgeIdx < 3; ++edgeIdx)
        {
            auto edge = tri.getEdge(edgeIdx);

            // Skip edges that share a node with the constraint edge
            if (edge[0] == nodeId1 || edge[0] == nodeId2 ||
                edge[1] == nodeId1 || edge[1] == nodeId2)
            {
                continue;
            }

            const Point2D& e1 = nodeCoords.at(edge[0]);
            const Point2D& e2 = nodeCoords.at(edge[1]);

            if (segmentsIntersect(p1, p2, e1, e2))
            {
                intersects = true;
                break;
            }
        }

        if (intersects)
        {
            result.push_back(i);
        }
    }

    return result;
}

void MeshOperations2D::retriangulate(size_t vertexNodeId,
                                     const std::vector<std::array<size_t, 2>>& boundary,
                                     std::vector<TriangleElement>& activeTriangles) const
{
    for (const auto& edge : boundary)
    {
        activeTriangles.emplace_back(std::array<size_t, 3>{vertexNodeId, edge[0], edge[1]});
    }
}

bool MeshOperations2D::segmentsIntersect(const Point2D& a1, const Point2D& a2,
                                         const Point2D& b1, const Point2D& b2) const
{
    // Compute orientation of ordered triplet (p, q, r)
    // Returns: 0 -> colinear, 1 -> clockwise, 2 -> counterclockwise
    auto orientation = [](const Point2D& p, const Point2D& q, const Point2D& r) -> int {
        double val = (q.y() - p.y()) * (r.x() - q.x()) -
                     (q.x() - p.x()) * (r.y() - q.y());

        if (std::abs(val) < 1e-10) return 0;
        return (val > 0) ? 1 : 2;
    };

    // Check if point q lies on segment pr
    auto onSegment = [](const Point2D& p, const Point2D& q, const Point2D& r) -> bool {
        return q.x() <= std::max(p.x(), r.x()) && q.x() >= std::min(p.x(), r.x()) &&
               q.y() <= std::max(p.y(), r.y()) && q.y() >= std::min(p.y(), r.y());
    };

    int o1 = orientation(a1, a2, b1);
    int o2 = orientation(a1, a2, b2);
    int o3 = orientation(b1, b2, a1);
    int o4 = orientation(b1, b2, a2);

    // General case
    if (o1 != o2 && o3 != o4)
        return true;

    // Special cases for collinear points
    if (o1 == 0 && onSegment(a1, b1, a2)) return true;
    if (o2 == 0 && onSegment(a1, b2, a2)) return true;
    if (o3 == 0 && onSegment(b1, a1, b2)) return true;
    if (o4 == 0 && onSegment(b1, a2, b2)) return true;

    return false;
}

} // namespace Meshing
