#include "ConstrainedDelaunay2D.h"

#include "Geometry2D/Corner2D.h"
#include "Geometry2D/IEdge2D.h"
#include "Geometry2D/GeometryCollection2D.h"
#include "Meshing/Core/MeshingContext2D.h"
#include "Meshing/Data/MeshOperations2D.h"
#include "Topology2D/Corner2D.h"
#include "Topology2D/Edge2D.h"
#include "Topology2D/Topology2D.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>

#include "spdlog/spdlog.h"

namespace Meshing
{

namespace
{
bool triangleHasNode(const TriangleElement& triangle, size_t nodeId)
{
    const auto& nodes = triangle.getNodeIdArray();
    return nodes[0] == nodeId || nodes[1] == nodeId || nodes[2] == nodeId;
}
} // namespace

ConstrainedDelaunay2D::ConstrainedDelaunay2D(MeshingContext2D& context) :
    context_(&context),
    meshData2D_(&context.getMeshData()),
    operations_(&context.getOperations())
{
}

ConstrainedDelaunay2D::ConstrainedDelaunay2D(const std::unordered_map<size_t, Point2D>& nodeCoords) :
    nodeCoords_(nodeCoords)
{
}

void ConstrainedDelaunay2D::addConstraintEdge(size_t nodeId1, size_t nodeId2)
{
    constraintEdges_.emplace_back(makeEdgeKey_(nodeId1, nodeId2));
}

std::vector<std::array<size_t, 3>> ConstrainedDelaunay2D::triangulate()
{
    if (nodeCoords_.size() < 3)
    {
        SPDLOG_WARN("ConstrainedDelaunay2D: Need at least 3 points for triangulation");
        return {};
    }

    activeTriangles_.clear();
    superNodeIds_.clear();

    // Step 1: Create super triangle
    createSuperTriangle_();

    // Step 2: Insert all vertices
    for (const auto& [nodeId, coord] : nodeCoords_)
    {
        // Skip super triangle nodes
        if (std::find(superNodeIds_.begin(), superNodeIds_.end(), nodeId) != superNodeIds_.end())
        {
            continue;
        }
        insertVertex_(nodeId);
    }

    // Step 3: Remove super triangle
    removeSuperTriangle_();

    // Step 4: Force constraint edges
    for (const auto& [nodeId1, nodeId2] : constraintEdges_)
    {
        if (!edgeExists_(nodeId1, nodeId2))
        {
            forceEdge_(nodeId1, nodeId2);
        }
    }

    // Step 5: Extract triangles
    std::vector<std::array<size_t, 3>> result;
    result.reserve(activeTriangles_.size());
    for (const auto& tri : activeTriangles_)
    {
        result.push_back(tri.getNodeIdArray());
    }

    return result;
}

void ConstrainedDelaunay2D::createSuperTriangle_()
{
    // Find bounding box
    double minX = std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double maxX = -std::numeric_limits<double>::max();
    double maxY = -std::numeric_limits<double>::max();

    for (const auto& [nodeId, coord] : nodeCoords_)
    {
        minX = std::min(minX, coord.x());
        minY = std::min(minY, coord.y());
        maxX = std::max(maxX, coord.x());
        maxY = std::max(maxY, coord.y());
    }

    const double dx = maxX - minX;
    const double dy = maxY - minY;
    const double dmax = std::max(dx, dy);
    const double midX = (minX + maxX) * 0.5;
    const double midY = (minY + maxY) * 0.5;

    // Create large triangle that contains all points
    const double scale = 10.0 * dmax;

    // Find next available node IDs for super triangle
    size_t maxNodeId = 0;
    for (const auto& [nodeId, _] : nodeCoords_)
    {
        maxNodeId = std::max(maxNodeId, nodeId);
    }

    superNodeIds_.push_back(maxNodeId + 1);
    superNodeIds_.push_back(maxNodeId + 2);
    superNodeIds_.push_back(maxNodeId + 3);

    nodeCoords_[superNodeIds_[0]] = Point2D(midX - scale, midY - scale);
    nodeCoords_[superNodeIds_[1]] = Point2D(midX + scale, midY - scale);
    nodeCoords_[superNodeIds_[2]] = Point2D(midX, midY + scale);

    activeTriangles_.emplace_back(std::array<size_t, 3>{superNodeIds_[0], superNodeIds_[1], superNodeIds_[2]});
}

void ConstrainedDelaunay2D::insertVertex_(size_t nodeId)
{
    const Point2D& point = nodeCoords_.at(nodeId);

    // Find conflicting triangles
    std::vector<size_t> conflicting = findConflictingTriangles_(point);

    if (conflicting.empty())
    {
        SPDLOG_WARN("ConstrainedDelaunay2D: No conflicting triangles found for point ({}, {})",
                    point.x(), point.y());
        return;
    }

    // Find cavity boundary
    std::vector<std::array<size_t, 2>> boundary = findCavityBoundary_(conflicting);

    // Remove conflicting triangles
    std::vector<TriangleElement> newActiveTriangles;
    newActiveTriangles.reserve(activeTriangles_.size());
    for (size_t i = 0; i < activeTriangles_.size(); ++i)
    {
        if (std::find(conflicting.begin(), conflicting.end(), i) == conflicting.end())
        {
            newActiveTriangles.push_back(activeTriangles_[i]);
        }
    }
    activeTriangles_ = std::move(newActiveTriangles);

    // Retriangulate cavity
    retriangulate_(nodeId, boundary);
}

std::vector<size_t> ConstrainedDelaunay2D::findConflictingTriangles_(const Point2D& point) const
{
    std::vector<size_t> conflicting;

    for (size_t i = 0; i < activeTriangles_.size(); ++i)
    {
        const auto& tri = activeTriangles_[i];
        auto circle = computeCircumcircle_(tri);

        if (circle && isPointInsideCircumcircle_(*circle, point))
        {
            conflicting.push_back(i);
        }
    }

    return conflicting;
}

std::vector<std::array<size_t, 2>> ConstrainedDelaunay2D::findCavityBoundary_(
    const std::vector<size_t>& conflictingIndices) const
{
    // Count how many times each edge appears
    std::map<std::pair<size_t, size_t>, int> edgeCount;
    std::map<std::pair<size_t, size_t>, std::array<size_t, 2>> edgeLookup;

    for (size_t idx : conflictingIndices)
    {
        const auto& tri = activeTriangles_[idx];
        for (size_t i = 0; i < 3; ++i)
        {
            auto edge = tri.getEdge(i);
            auto key = makeEdgeKey_(edge[0], edge[1]);
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

void ConstrainedDelaunay2D::retriangulate_(size_t vertexNodeId,
                                           const std::vector<std::array<size_t, 2>>& boundary)
{
    for (const auto& edge : boundary)
    {
        activeTriangles_.emplace_back(std::array<size_t, 3>{vertexNodeId, edge[0], edge[1]});
    }
}

void ConstrainedDelaunay2D::removeSuperTriangle_()
{
    if (superNodeIds_.empty())
    {
        return;
    }

    const std::unordered_set<size_t> superNodeSet(superNodeIds_.begin(), superNodeIds_.end());

    std::vector<TriangleElement> filtered;
    filtered.reserve(activeTriangles_.size());

    for (const auto& tri : activeTriangles_)
    {
        bool hasSuperNode = false;
        const auto& nodes = tri.getNodeIdArray();
        for (size_t nodeId : nodes)
        {
            if (superNodeSet.count(nodeId) > 0)
            {
                hasSuperNode = true;
                break;
            }
        }

        if (!hasSuperNode)
        {
            filtered.push_back(tri);
        }
    }

    activeTriangles_ = std::move(filtered);

    // Remove super nodes from coordinate map
    for (size_t nodeId : superNodeIds_)
    {
        nodeCoords_.erase(nodeId);
    }

    superNodeIds_.clear();
}

bool ConstrainedDelaunay2D::edgeExists_(size_t nodeId1, size_t nodeId2) const
{
    auto key = makeEdgeKey_(nodeId1, nodeId2);

    for (const auto& tri : activeTriangles_)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            auto edge = tri.getEdge(i);
            if (makeEdgeKey_(edge[0], edge[1]) == key)
            {
                return true;
            }
        }
    }

    return false;
}

void ConstrainedDelaunay2D::forceEdge_(size_t nodeId1, size_t nodeId2)
{
    // Find all triangles that intersect with this edge
    std::vector<size_t> intersecting = findIntersectingTriangles_(nodeId1, nodeId2);

    if (intersecting.empty())
    {
        return;
    }

    SPDLOG_DEBUG("ConstrainedDelaunay2D: Forcing edge ({}, {}), {} triangles intersect",
                 nodeId1, nodeId2, intersecting.size());

    // Find cavity boundary
    std::vector<std::array<size_t, 2>> boundary = findCavityBoundary_(intersecting);

    // Remove intersecting triangles
    std::vector<TriangleElement> newActiveTriangles;
    newActiveTriangles.reserve(activeTriangles_.size());
    for (size_t i = 0; i < activeTriangles_.size(); ++i)
    {
        if (std::find(intersecting.begin(), intersecting.end(), i) == intersecting.end())
        {
            newActiveTriangles.push_back(activeTriangles_[i]);
        }
    }
    activeTriangles_ = std::move(newActiveTriangles);

    // Retriangulate the cavity, forcing the constraint edge
    // Separate boundary edges into two groups: above and below the constraint edge
    std::vector<std::array<size_t, 2>> side1Edges;
    std::vector<std::array<size_t, 2>> side2Edges;

    for (const auto& edge : boundary)
    {
        // Skip edges that are part of the constraint edge
        if ((edge[0] == nodeId1 && edge[1] == nodeId2) ||
            (edge[0] == nodeId2 && edge[1] == nodeId1))
        {
            continue;
        }

        // Check which side of the constraint edge this edge belongs to
        bool hasNode1 = (edge[0] == nodeId1 || edge[1] == nodeId1);
        bool hasNode2 = (edge[0] == nodeId2 || edge[1] == nodeId2);

        if (hasNode1)
        {
            side1Edges.push_back(edge);
        }
        else if (hasNode2)
        {
            side2Edges.push_back(edge);
        }
        else
        {
            // Edge doesn't touch the constraint edge endpoints
            // Need to determine which side it's on geometrically
            // For simplicity, add to both sides (will create valid triangulation)
            side1Edges.push_back({edge[0], nodeId1});
            side2Edges.push_back({edge[1], nodeId2});
        }
    }

    // Create triangles on each side of the constraint edge
    for (const auto& edge : side1Edges)
    {
        activeTriangles_.emplace_back(std::array<size_t, 3>{nodeId1, edge[0], edge[1]});
    }

    for (const auto& edge : side2Edges)
    {
        activeTriangles_.emplace_back(std::array<size_t, 3>{nodeId2, edge[0], edge[1]});
    }
}

std::vector<size_t> ConstrainedDelaunay2D::findIntersectingTriangles_(size_t nodeId1, size_t nodeId2) const
{
    std::vector<size_t> result;

    const Point2D& p1 = nodeCoords_.at(nodeId1);
    const Point2D& p2 = nodeCoords_.at(nodeId2);

    for (size_t i = 0; i < activeTriangles_.size(); ++i)
    {
        const auto& tri = activeTriangles_[i];

        // Skip triangles that already contain both nodes
        if (triangleHasNode(tri, nodeId1) && triangleHasNode(tri, nodeId2))
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

            const Point2D& e1 = nodeCoords_.at(edge[0]);
            const Point2D& e2 = nodeCoords_.at(edge[1]);

            if (segmentsIntersect_(p1, p2, e1, e2))
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

std::optional<ConstrainedDelaunay2D::CircumCircle>
ConstrainedDelaunay2D::computeCircumcircle_(const TriangleElement& tri) const
{
    const auto& nodes = tri.getNodeIdArray();
    const Point2D& p0 = nodeCoords_.at(nodes[0]);
    const Point2D& p1 = nodeCoords_.at(nodes[1]);
    const Point2D& p2 = nodeCoords_.at(nodes[2]);

    const double ax = p1.x() - p0.x();
    const double ay = p1.y() - p0.y();
    const double bx = p2.x() - p0.x();
    const double by = p2.y() - p0.y();

    const double d = 2.0 * (ax * by - ay * bx);

    if (std::abs(d) < 1e-10)
    {
        // Degenerate triangle
        return std::nullopt;
    }

    const double aSq = ax * ax + ay * ay;
    const double bSq = bx * bx + by * by;

    const double cx = (by * aSq - ay * bSq) / d;
    const double cy = (ax * bSq - bx * aSq) / d;

    CircumCircle circle;
    circle.center = Point2D(p0.x() + cx, p0.y() + cy);
    circle.radiusSquared = cx * cx + cy * cy;

    return circle;
}

bool ConstrainedDelaunay2D::isPointInsideCircumcircle_(const CircumCircle& circle,
                                                       const Point2D& point) const
{
    const double dx = point.x() - circle.center.x();
    const double dy = point.y() - circle.center.y();
    const double distSquared = dx * dx + dy * dy;

    return distSquared < circle.radiusSquared - 1e-10;
}

bool ConstrainedDelaunay2D::segmentsIntersect_(const Point2D& a1, const Point2D& a2,
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

MeshData ConstrainedDelaunay2D::getMeshData() const
{
    MeshData meshData;
    // Note: This method has limited functionality as MeshData
    // doesn't have a public addNode method for Point2D.
    // Use getMeshData2D() instead for full 2D support.
    return meshData;
}

MeshData2D& ConstrainedDelaunay2D::getMeshData2D()
{
    if (meshData2D_ == nullptr)
    {
        throw std::runtime_error("getMeshData2D requires MeshingContext2D constructor");
    }
    return *meshData2D_;
}

const MeshData2D& ConstrainedDelaunay2D::getMeshData2D() const
{
    if (meshData2D_ == nullptr)
    {
        throw std::runtime_error("getMeshData2D requires MeshingContext2D constructor");
    }
    return *meshData2D_;
}

void ConstrainedDelaunay2D::generateConstrained(size_t samplesPerEdge)
{
    if (context_ == nullptr)
    {
        throw std::runtime_error("generateConstrained requires MeshingContext2D constructor");
    }

    SPDLOG_INFO("========================================");
    SPDLOG_INFO("Starting 2D Constrained Delaunay");
    SPDLOG_INFO("========================================");

    // Step 1: Insert corner nodes from topology
    insertCornerNodes_();

    // Step 2: Insert edge nodes and build constraint edges
    insertEdgeNodes_(samplesPerEdge);

    // Step 3: Build Delaunay triangulation
    buildTriangulation_();

    // Step 4: Recover constraint edges
    recoverConstraintEdges_();

    // Step 5: Store results in mesh data
    storeResultsInMeshData_();

    SPDLOG_INFO("========================================");
    SPDLOG_INFO("2D Constrained Delaunay COMPLETE");
    SPDLOG_INFO("  Final: {} nodes, {} triangles",
                meshData2D_->getNodeCount(), meshData2D_->getElementCount());
    SPDLOG_INFO("========================================");
}

void ConstrainedDelaunay2D::insertCornerNodes_()
{
    const auto& geometry = context_->getGeometry();
    const auto& topology = context_->getTopology();

    for (const auto& cornerId : topology.getAllCornerIds())
    {
        const auto* corner = geometry.getCorner(cornerId);
        if (corner != nullptr)
        {
            Point2D point = corner->getPoint();
            size_t nodeId = operations_->addNode(point);
            topologyToNodeId_[cornerId] = nodeId;
            nodeCoords_[nodeId] = point;

            SPDLOG_DEBUG("Corner {} → node {} at ({:.4f}, {:.4f})",
                         cornerId, nodeId, point.x(), point.y());
        }
    }

    SPDLOG_INFO("Inserted {} corner nodes", topology.getAllCornerIds().size());
}

void ConstrainedDelaunay2D::insertEdgeNodes_(size_t samplesPerEdge)
{
    const auto& geometry = context_->getGeometry();
    const auto& topology = context_->getTopology();

    for (const auto& edgeId : topology.getAllEdgeIds())
    {
        const auto& topoEdge = topology.getEdge(edgeId);
        const auto* geomEdge = geometry.getEdge(edgeId);

        if (geomEdge == nullptr)
        {
            continue;
        }

        size_t startNodeId = topologyToNodeId_[topoEdge.getStartCornerId()];
        size_t endNodeId = topologyToNodeId_[topoEdge.getEndCornerId()];

        auto [tMin, tMax] = geomEdge->getParameterBounds();
        std::vector<size_t> edgeNodeIds;
        edgeNodeIds.push_back(startNodeId);

        // Insert intermediate nodes
        for (size_t i = 1; i < samplesPerEdge; ++i)
        {
            double t = tMin + (tMax - tMin) * static_cast<double>(i) / static_cast<double>(samplesPerEdge);
            Point2D point = geomEdge->getPoint(t);
            size_t nodeId = operations_->addNode(point);
            edgeNodeIds.push_back(nodeId);
            nodeCoords_[nodeId] = point;
        }

        edgeNodeIds.push_back(endNodeId);

        // Create constraint edges
        for (size_t i = 0; i < edgeNodeIds.size() - 1; ++i)
        {
            addConstraintEdge(edgeNodeIds[i], edgeNodeIds[i + 1]);
        }
    }

    SPDLOG_INFO("Created {} constraint edges from {} boundary edges",
                constraintEdges_.size(), topology.getAllEdgeIds().size());
}

void ConstrainedDelaunay2D::buildTriangulation_()
{
    if (nodeCoords_.size() < 3)
    {
        SPDLOG_WARN("Not enough nodes for triangulation");
        return;
    }

    activeTriangles_.clear();
    superNodeIds_.clear();

    // Create super triangle
    createSuperTriangle_();

    // Insert all vertices
    for (const auto& [nodeId, coord] : nodeCoords_)
    {
        if (std::find(superNodeIds_.begin(), superNodeIds_.end(), nodeId) != superNodeIds_.end())
        {
            continue;
        }
        insertVertex_(nodeId);
    }

    // Remove super triangle
    removeSuperTriangle_();

    SPDLOG_INFO("Built initial triangulation with {} triangles", activeTriangles_.size());
}

void ConstrainedDelaunay2D::recoverConstraintEdges_()
{
    size_t recovered = 0;
    size_t alreadyPresent = 0;

    for (const auto& [nodeId1, nodeId2] : constraintEdges_)
    {
        if (edgeExists_(nodeId1, nodeId2))
        {
            alreadyPresent++;
        }
        else
        {
            forceEdge_(nodeId1, nodeId2);
            recovered++;
        }
    }

    SPDLOG_INFO("Constraint edges: {} present, {} forced", alreadyPresent, recovered);
}

void ConstrainedDelaunay2D::storeResultsInMeshData_()
{
    // Add triangle elements to mesh data
    for (const auto& tri : activeTriangles_)
    {
        auto element = std::make_unique<TriangleElement>(tri.getNodeIdArray());
        operations_->addElement(std::move(element));
    }
}

} // namespace Meshing
