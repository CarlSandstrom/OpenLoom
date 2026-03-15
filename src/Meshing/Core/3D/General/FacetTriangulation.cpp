#include "Meshing/Core/3D/General/FacetTriangulation.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Core/2D/ConstrainedDelaunay2D.h"
#include "Meshing/Core/2D/MeshOperations2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/Node2D.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include "Topology/Surface3D.h"
#include "Topology/Topology3D.h"
#include "spdlog/spdlog.h"

namespace Meshing
{

FacetTriangulation::FacetTriangulation(
    const Geometry3D::ISurface3D& surface,
    const Topology3D::Surface3D& topoSurface,
    const Topology3D::Topology3D& fullTopology,
    const Geometry3D::GeometryCollection3D& fullGeometry) :
    surfaceId_(topoSurface.getId()),
    surface_(&surface)
{
    // Create 2D meshing context from the surface
    context_ = std::make_unique<MeshingContext2D>(MeshingContext2D::fromSurface(surface, topoSurface, fullTopology, fullGeometry));
}

FacetTriangulation::~FacetTriangulation() = default;

FacetTriangulation::FacetTriangulation(FacetTriangulation&&) noexcept = default;
FacetTriangulation& FacetTriangulation::operator=(FacetTriangulation&&) noexcept = default;

void FacetTriangulation::initialize(DiscretizationResult2D discretization2D,
                                    std::vector<size_t> localIndexToNode3DId)
{
    node3DTo2DMap_.clear();
    node2DTo3DMap_.clear();
    edgeIdToNode2DSeq_.clear();

    // Run constrained Delaunay: registers boundary segments, enforces them,
    // and removes exterior triangles.
    ConstrainedDelaunay2D constrained(*context_, discretization2D, {}, surfaceId_);
    constrained.triangulate();

    // Build bidirectional 3D↔2D node mappings
    const auto& pointIndexToNode2DMap = constrained.getPointIndexToNodeIdMap();

    for (size_t localIndex = 0; localIndex < localIndexToNode3DId.size(); ++localIndex)
    {
        size_t node3DId = localIndexToNode3DId[localIndex];
        auto it = pointIndexToNode2DMap.find(localIndex);
        if (it != pointIndexToNode2DMap.end())
        {
            size_t node2DId = it->second;
            node3DTo2DMap_[node3DId] = node2DId;
            node2DTo3DMap_[node2DId] = node3DId;
        }
    }

    // Build edge→node2D sequence map for TwinManager construction.
    // Translates each edge's local-index sequence into the corresponding 2D node IDs.
    for (const auto& [edgeId, localIndices] : discretization2D.edgeIdToPointIndicesMap)
    {
        std::vector<size_t> node2DIds;
        node2DIds.reserve(localIndices.size());
        for (size_t localIdx : localIndices)
        {
            auto it = pointIndexToNode2DMap.find(localIdx);
            if (it != pointIndexToNode2DMap.end())
                node2DIds.push_back(it->second);
        }
        if (!node2DIds.empty())
            edgeIdToNode2DSeq_[edgeId] = std::move(node2DIds);
    }

    spdlog::debug("FacetTriangulation for surface {}: {} points, {} triangles, {} node mappings",
                  surfaceId_, discretization2D.points.size(),
                  context_->getMeshData().getElementCount(), node3DTo2DMap_.size());
}

std::vector<Point3D> FacetTriangulation::resolveRefinementNodes(size_t& nextNode3DId)
{
    std::vector<Point3D> newPoints;
    const auto& meshData = context_->getMeshData();
    for (const auto& [node2DId, node2D] : meshData.getNodes())
    {
        if (node2DTo3DMap_.find(node2DId) == node2DTo3DMap_.end())
        {
            const auto& uv = node2D->getCoordinates();
            Point3D pt3D = surface_->getPoint(uv.x(), uv.y());
            size_t node3DId = nextNode3DId++;
            node2DTo3DMap_[node2DId] = node3DId;
            node3DTo2DMap_[node3DId] = node2DId;
            newPoints.push_back(pt3D);
        }
    }
    spdlog::debug("FacetTriangulation {}: resolved {} refinement nodes", surfaceId_, newPoints.size());
    return newPoints;
}

std::vector<ConstrainedSubfacet3D> FacetTriangulation::getSubfacets() const
{
    std::vector<ConstrainedSubfacet3D> subfacets;

    const auto& meshData = context_->getMeshData();
    for (const auto& [elemId, element] : meshData.getElements())
    {
        const auto* triangle = dynamic_cast<const TriangleElement*>(element.get());
        if (!triangle)
        {
            continue;
        }

        const auto& nodeIds2D = triangle->getNodeIds();

        // Convert 2D node IDs to 3D node IDs
        auto node3D_0 = get3DNodeId(nodeIds2D[0]);
        auto node3D_1 = get3DNodeId(nodeIds2D[1]);
        auto node3D_2 = get3DNodeId(nodeIds2D[2]);

        if (node3D_0 && node3D_1 && node3D_2)
        {
            subfacets.push_back(ConstrainedSubfacet3D{
                *node3D_0,
                *node3D_1,
                *node3D_2,
                surfaceId_});
        }
        else
        {
            spdlog::warn("FacetTriangulation {}: Could not map 2D triangle ({}, {}, {}) to 3D node IDs",
                         surfaceId_, nodeIds2D[0], nodeIds2D[1], nodeIds2D[2]);
        }
    }

    return subfacets;
}

bool FacetTriangulation::insertVertex(size_t node3DId, const Point2D& uvCoords)
{
    // Check if node already exists
    if (node3DTo2DMap_.contains(node3DId))
    {
        spdlog::debug("FacetTriangulation {}: Node {} already exists", surfaceId_, node3DId);
        return true;
    }

    // Insert vertex using Bowyer-Watson
    auto& operations = context_->getOperations();
    size_t node2DId = operations.insertVertexBowyerWatson(uvCoords);

    // Update mappings
    node3DTo2DMap_[node3DId] = node2DId;
    node2DTo3DMap_[node2DId] = node3DId;

    spdlog::debug("FacetTriangulation {}: Inserted vertex {} (3D) -> {} (2D) at ({}, {})",
                  surfaceId_, node3DId, node2DId, uvCoords.x(), uvCoords.y());

    return true;
}

std::optional<size_t> FacetTriangulation::get2DNodeId(size_t node3DId) const
{
    auto it = node3DTo2DMap_.find(node3DId);
    if (it != node3DTo2DMap_.end())
    {
        return it->second;
    }
    return std::nullopt;
}

std::optional<size_t> FacetTriangulation::get3DNodeId(size_t node2DId) const
{
    auto it = node2DTo3DMap_.find(node2DId);
    if (it != node2DTo3DMap_.end())
    {
        return it->second;
    }
    return std::nullopt;
}

const std::vector<size_t>* FacetTriangulation::getEdgeNodeSequence(const std::string& edgeId) const
{
    auto it = edgeIdToNode2DSeq_.find(edgeId);
    if (it != edgeIdToNode2DSeq_.end())
        return &it->second;
    return nullptr;
}

void FacetTriangulation::registerNode(size_t node2DId, size_t node3DId)
{
    node2DTo3DMap_[node2DId] = node3DId;
    node3DTo2DMap_[node3DId] = node2DId;
}

void FacetTriangulation::updateEdgeNodeAfterSplit(size_t nodeId1, size_t nodeId2, size_t midNodeId)
{
    for (auto& [edgeId, nodeSeq] : edgeIdToNode2DSeq_)
    {
        for (size_t index = 0; index + 1 < nodeSeq.size(); ++index)
        {
            bool forwardMatch = (nodeSeq[index] == nodeId1 && nodeSeq[index + 1] == nodeId2);
            bool reverseMatch = (nodeSeq[index] == nodeId2 && nodeSeq[index + 1] == nodeId1);
            if (forwardMatch || reverseMatch)
            {
                nodeSeq.insert(nodeSeq.begin() + static_cast<std::ptrdiff_t>(index) + 1, midNodeId);
                spdlog::debug("FacetTriangulation {}: updated edge '{}' sequence with mid node {} between ({}, {})",
                              surfaceId_, edgeId, midNodeId, nodeId1, nodeId2);
                return;
            }
        }
    }

    spdlog::warn("FacetTriangulation {}: updateEdgeNodeAfterSplit: segment ({}, {}) not found in any edge node sequence",
                 surfaceId_, nodeId1, nodeId2);
}

} // namespace Meshing
