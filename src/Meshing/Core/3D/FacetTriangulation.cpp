#include "FacetTriangulation.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Core/2D/Delaunay2D.h"
#include "Meshing/Core/2D/MeshOperations2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
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
    context_ = std::make_unique<MeshingContext2D>(
        MeshingContext2D::fromSurface(surface, topoSurface, fullTopology, fullGeometry));
}

FacetTriangulation::~FacetTriangulation() = default;

FacetTriangulation::FacetTriangulation(FacetTriangulation&&) noexcept = default;
FacetTriangulation& FacetTriangulation::operator=(FacetTriangulation&&) noexcept = default;

void FacetTriangulation::initialize(const std::map<size_t, Point2D>& node3DToPoint2DMap)
{
    // Clear existing mappings
    node3DTo2DMap_.clear();
    node2DTo3DMap_.clear();

    // Build vectors for Delaunay2D
    std::vector<Point2D> points;
    std::vector<size_t> node3DIds;

    points.reserve(node3DToPoint2DMap.size());
    node3DIds.reserve(node3DToPoint2DMap.size());

    for (const auto& [node3DId, uvCoord] : node3DToPoint2DMap)
    {
        points.push_back(uvCoord);
        node3DIds.push_back(node3DId);
    }

    // Run 2D Delaunay triangulation
    auto& meshData = context_->getMeshData();
    Delaunay2D delaunay(points, &meshData);
    delaunay.triangulate();

    // Build bidirectional mappings from point indices to node IDs
    auto pointIndexToNode2DMap = delaunay.getPointIndexToNodeIdMap();

    for (size_t pointIdx = 0; pointIdx < node3DIds.size(); ++pointIdx)
    {
        size_t node3DId = node3DIds[pointIdx];
        auto node2DIt = pointIndexToNode2DMap.find(pointIdx);

        if (node2DIt != pointIndexToNode2DMap.end())
        {
            size_t node2DId = node2DIt->second;
            node3DTo2DMap_[node3DId] = node2DId;
            node2DTo3DMap_[node2DId] = node3DId;
        }
    }

    spdlog::debug("FacetTriangulation for surface {}: {} points, {} triangles, {} node mappings",
                  surfaceId_, points.size(), meshData.getElementCount(), node3DTo2DMap_.size());
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

} // namespace Meshing
