#include "MeshOperations2D.h"

#include "Computer2D.h"
#include "Meshing/Data/MeshMutator2D.h"

#include <algorithm>
#include <cmath>
#include <map>

#include "spdlog/spdlog.h"

namespace Meshing
{

MeshOperations2D::MeshOperations2D(MeshData2D& meshData) :
    meshData_(meshData),
    mutator_(std::make_unique<MeshMutator2D>(meshData)),
    computer_(std::make_unique<Computer2D>(meshData))
{
}

void MeshOperations2D::insertVertexBowyerWatson(size_t nodeId,
                                                const std::unordered_map<size_t, Point2D>& nodeCoords,
                                                std::vector<TriangleElement>& activeTriangles) const
{
    const Point2D& point = nodeCoords.at(nodeId);

    // Find conflicting triangles
    std::vector<size_t> conflicting = findConflictingTriangles(point);

    if (conflicting.empty())
    {
        SPDLOG_WARN("MeshOperations2D: No conflicting triangles found for point ({}, {})",
                    point.x(), point.y());
        return;
    }

    // Find cavity boundary
    std::vector<std::array<size_t, 2>> boundary = findCavityBoundary(conflicting);

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

size_t MeshOperations2D::insertVertexBowyerWatson(const Point2D& point)
{
    std::vector<size_t> conflicting = findConflictingTriangles(point);

    if (conflicting.empty())
    {
        SPDLOG_WARN("MeshOperations2D: No conflicting triangles found for point ({}, {})",
                    point.x(), point.y());
        return -1;
    }

    std::vector<std::array<size_t, 2>> boundary = findCavityBoundary(conflicting);

    for (auto conflictingElementId : conflicting)
    {
        mutator_->removeElement(conflictingElementId);
    }

    size_t newVertex = mutator_->addNode(point);

    for (const auto& edge : boundary)
    {
        auto newTriangle = std::make_unique<TriangleElement>(std::array<size_t, 3>{newVertex, edge[0], edge[1]});
        mutator_->addElement(std::move(newTriangle));
    }

    return newVertex; // Placeholder return value
}

std::vector<size_t> MeshOperations2D::findConflictingTriangles(const Point2D& point) const
{
    std::vector<size_t> conflicting;

    for (const auto& [id, element] : meshData_.getElements())
    {
        const auto* triangle = dynamic_cast<const TriangleElement*>(element.get());
        if (triangle == nullptr)
        {
            SPDLOG_WARN("MeshOperations2D: Skipping non-triangle element {}", id);
            continue;
        }

        auto circle = computer_->computeCircumcircle(*triangle);

        if (circle && Computer2D::isPointInsideCircumcircle(*circle, point))
        {
            conflicting.push_back(id);
        }
    }

    return conflicting;
}

std::vector<std::array<size_t, 2>> MeshOperations2D::findCavityBoundary(const std::vector<size_t>& conflictingIndices) const
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

bool MeshOperations2D::removeTrianglesContainingNode(size_t nodeId)
{
    std::vector<size_t> elementsToRemove;
    for (const auto& [id, element] : meshData_.getElements())
    {
        const auto* triangle = dynamic_cast<const TriangleElement*>(element.get());
        if (triangle && triangle->getHasNode(nodeId))
        {
            elementsToRemove.push_back(id);
        }
    }

    for (size_t id : elementsToRemove)
    {
        mutator_->removeElement(id);
    }

    return !elementsToRemove.empty();
}

std::vector<size_t> MeshOperations2D::findIntersectingTriangles(size_t nodeId1, size_t nodeId2) const
{
    std::vector<size_t> result;

    const auto& p1 = meshData_.getNode(nodeId1)->getCoordinates();
    const auto& p2 = meshData_.getNode(nodeId2)->getCoordinates();

    for (auto& idAndTriangle : meshData_.getElements())
    {
        auto triangle = dynamic_cast<const TriangleElement*>(idAndTriangle.second.get());
        if (!triangle)
        {
            spdlog::warn("MeshOperations2D: Skipping non-triangle element {}", idAndTriangle.first);
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

bool MeshOperations2D::enforceEdge(size_t nodeId1, size_t nodeId2)
{

    auto intersectingTriangles = findIntersectingTriangles(nodeId1, nodeId2);

    if (intersectingTriangles.empty())
    {
        return true; // Edge already enforced
    }

    // Find the cavity boundary
    std::vector<std::array<size_t, 2>> boundary = findCavityBoundary(intersectingTriangles);

    // Remove intersecting triangles
    for (auto index : intersectingTriangles)
    {
        const auto& tri = *meshData_.getElement(index);
        mutator_->removeElement(index);
    }

    // Retriangulate the cavity

    return true;
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
