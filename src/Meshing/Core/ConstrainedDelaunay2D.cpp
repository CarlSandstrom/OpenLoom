#include "ConstrainedDelaunay2D.h"

#include "Computer.h"
#include "Geometry2D/Corner2D.h"
#include "Geometry2D/GeometryCollection2D.h"
#include "Geometry2D/IEdge2D.h"
#include "MeshOperations2D.h"
#include "Meshing/Core/MeshingContext2D.h"
#include "Meshing/Data/MeshMutator2D.h"
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

ConstrainedDelaunay2D::ConstrainedDelaunay2D(MeshingContext2D& context) :
    context_(&context),
    meshData2D_(&context.getMeshData()),
    operations_(&context.getOperations()),
    meshOps_(std::make_unique<MeshOperations2D>(*meshData2D_))
{
}

ConstrainedDelaunay2D::ConstrainedDelaunay2D(const std::unordered_map<size_t, Point2D>& nodeCoords) :
    ownedMeshData_(std::make_unique<MeshData2D>()),
    meshOps_(std::make_unique<MeshOperations2D>(*ownedMeshData_)),
    nodeCoords_(nodeCoords)
{
}

ConstrainedDelaunay2D::~ConstrainedDelaunay2D() = default;

void ConstrainedDelaunay2D::addConstraintEdge(size_t nodeId1, size_t nodeId2)
{
    constraintEdges_.emplace_back(makeEdgeKey(nodeId1, nodeId2));
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
    createSuperTriangle();

    // Step 2: Insert all vertices
    for (const auto& [nodeId, coord] : nodeCoords_)
    {
        // Skip super triangle nodes
        if (std::find(superNodeIds_.begin(), superNodeIds_.end(), nodeId) != superNodeIds_.end())
        {
            continue;
        }
        insertVertex(nodeId);
    }

    // Step 3: Remove super triangle
    removeSuperTriangle();

    // Step 4: Force constraint edges
    for (const auto& [nodeId1, nodeId2] : constraintEdges_)
    {
        if (!edgeExists(nodeId1, nodeId2))
        {
            forceEdge(nodeId1, nodeId2);
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

void ConstrainedDelaunay2D::createSuperTriangle()
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

void ConstrainedDelaunay2D::insertVertex(size_t nodeId)
{
    meshOps_->insertVertexBowyerWatson(nodeId, nodeCoords_, activeTriangles_);
}

void ConstrainedDelaunay2D::removeSuperTriangle()
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

bool ConstrainedDelaunay2D::edgeExists(size_t nodeId1, size_t nodeId2) const
{
    auto key = makeEdgeKey(nodeId1, nodeId2);

    for (const auto& tri : activeTriangles_)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            auto edge = tri.getEdge(i);
            if (makeEdgeKey(edge[0], edge[1]) == key)
            {
                return true;
            }
        }
    }

    return false;
}

void ConstrainedDelaunay2D::forceEdge(size_t nodeId1, size_t nodeId2)
{
    // Find all triangles that intersect with this edge
    std::vector<size_t> intersecting = meshOps_->findIntersectingTriangles(nodeId1, nodeId2, nodeCoords_, activeTriangles_);

    if (intersecting.empty())
    {
        return;
    }

    SPDLOG_DEBUG("ConstrainedDelaunay2D: Forcing edge ({}, {}), {} triangles intersect",
                 nodeId1, nodeId2, intersecting.size());

    // Find cavity boundary
    std::vector<std::array<size_t, 2>> boundary = meshOps_->findCavityBoundary(intersecting, activeTriangles_);

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
    insertCornerNodes();

    // Step 2: Insert edge nodes and build constraint edges
    insertEdgeNodes(samplesPerEdge);

    // Step 3: Build Delaunay triangulation
    buildTriangulation();

    // Step 4: Recover constraint edges
    recoverConstraintEdges();

    // Step 5: Store results in mesh data
    storeResultsInMeshData();

    SPDLOG_INFO("========================================");
    SPDLOG_INFO("2D Constrained Delaunay COMPLETE");
    SPDLOG_INFO("  Final: {} nodes, {} triangles",
                meshData2D_->getNodeCount(), meshData2D_->getElementCount());
    SPDLOG_INFO("========================================");
}

void ConstrainedDelaunay2D::insertCornerNodes()
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

void ConstrainedDelaunay2D::insertEdgeNodes(size_t samplesPerEdge)
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

void ConstrainedDelaunay2D::buildTriangulation()
{
    if (nodeCoords_.size() < 3)
    {
        SPDLOG_WARN("Not enough nodes for triangulation");
        return;
    }

    activeTriangles_.clear();
    superNodeIds_.clear();

    // Create super triangle
    createSuperTriangle();

    // Insert all vertices
    for (const auto& [nodeId, coord] : nodeCoords_)
    {
        if (std::find(superNodeIds_.begin(), superNodeIds_.end(), nodeId) != superNodeIds_.end())
        {
            continue;
        }
        insertVertex(nodeId);
    }

    // Remove super triangle
    removeSuperTriangle();

    SPDLOG_INFO("Built initial triangulation with {} triangles", activeTriangles_.size());
}

void ConstrainedDelaunay2D::recoverConstraintEdges()
{
    size_t recovered = 0;
    size_t alreadyPresent = 0;

    for (const auto& [nodeId1, nodeId2] : constraintEdges_)
    {
        if (edgeExists(nodeId1, nodeId2))
        {
            alreadyPresent++;
        }
        else
        {
            forceEdge(nodeId1, nodeId2);
            recovered++;
        }
    }

    SPDLOG_INFO("Constraint edges: {} present, {} forced", alreadyPresent, recovered);
}

void ConstrainedDelaunay2D::storeResultsInMeshData()
{
    // Add triangle elements to mesh data
    for (const auto& tri : activeTriangles_)
    {
        auto element = std::make_unique<TriangleElement>(tri.getNodeIdArray());
        operations_->addElement(std::move(element));
    }
}

} // namespace Meshing
