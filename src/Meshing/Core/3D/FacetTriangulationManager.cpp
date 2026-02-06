#include "FacetTriangulationManager.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/Node3D.h"
#include "Topology/Surface3D.h"
#include "Topology/Topology3D.h"
#include "spdlog/spdlog.h"
#include <set>

namespace Meshing
{

FacetTriangulationManager::FacetTriangulationManager(
    const Geometry3D::GeometryCollection3D& geometry,
    const Topology3D::Topology3D& topology) :
    geometry_(&geometry),
    topology_(&topology)
{
}

FacetTriangulationManager::~FacetTriangulationManager() = default;

FacetTriangulationManager::FacetTriangulationManager(FacetTriangulationManager&&) noexcept = default;
FacetTriangulationManager& FacetTriangulationManager::operator=(FacetTriangulationManager&&) noexcept = default;

void FacetTriangulationManager::initializeFromDiscretization(
    const DiscretizationResult3D& discretization,
    const std::map<size_t, size_t>& pointIndexToNodeIdMap,
    const MeshData3D& meshData)
{
    // Clear existing triangulations
    facetTriangulations_.clear();

    // Create a facet triangulation for each surface
    for (const auto& surfaceId : topology_->getAllSurfaceIds())
    {
        const auto& topoSurface = topology_->getSurface(surfaceId);
        auto* surface = geometry_->getSurface(surfaceId);

        if (!surface)
        {
            spdlog::warn("FacetTriangulationManager: No geometry for surface {}", surfaceId);
            continue;
        }

        // Create the facet triangulation
        auto facetTriang = std::make_unique<FacetTriangulation>(
            *surface, topoSurface, *topology_, *geometry_);

        // Collect all point indices for this surface
        std::vector<size_t> surfacePointIndices = collectSurfacePointIndices(surfaceId, discretization);

        // Build the map from 3D node IDs to (u,v) coordinates
        std::map<size_t, Point2D> node3DToPoint2DMap;

        for (size_t pointIdx : surfacePointIndices)
        {
            auto nodeIdIt = pointIndexToNodeIdMap.find(pointIdx);
            if (nodeIdIt == pointIndexToNodeIdMap.end())
            {
                spdlog::warn("FacetTriangulationManager: Point index {} not found in pointIndexToNodeIdMap",
                             pointIdx);
                continue;
            }

            size_t node3DId = nodeIdIt->second;
            const auto* node = meshData.getNode(node3DId);
            if (!node)
            {
                spdlog::warn("FacetTriangulationManager: Node {} not found in meshData", node3DId);
                continue;
            }

            // Project 3D point to (u,v) parametric coordinates
            Point2D uvCoord = surface->projectPoint(node->getCoordinates());
            node3DToPoint2DMap[node3DId] = uvCoord;
        }

        // Initialize the triangulation
        facetTriang->initialize(node3DToPoint2DMap);

        spdlog::debug("FacetTriangulationManager: Created triangulation for surface {} with {} points",
                      surfaceId, node3DToPoint2DMap.size());

        facetTriangulations_[surfaceId] = std::move(facetTriang);
    }

    spdlog::info("FacetTriangulationManager: Initialized {} facet triangulations",
                 facetTriangulations_.size());
}

std::vector<size_t> FacetTriangulationManager::collectSurfacePointIndices(
    const std::string& surfaceId,
    const DiscretizationResult3D& discretization) const
{
    std::set<size_t> pointIndices;

    const auto& topoSurface = topology_->getSurface(surfaceId);

    // Add corner points
    for (const auto& cornerId : topoSurface.getCornerIds())
    {
        auto it = discretization.cornerIdToPointIndexMap.find(cornerId);
        if (it != discretization.cornerIdToPointIndexMap.end())
        {
            pointIndices.insert(it->second);
        }
    }

    // Add edge points (from boundary edges)
    for (const auto& edgeId : topoSurface.getBoundaryEdgeIds())
    {
        auto it = discretization.edgeIdToPointIndicesMap.find(edgeId);
        if (it != discretization.edgeIdToPointIndicesMap.end())
        {
            for (size_t idx : it->second)
            {
                pointIndices.insert(idx);
            }
        }
    }

    // Add interior surface points
    auto surfacePointsIt = discretization.surfaceIdToPointIndicesMap.find(surfaceId);
    if (surfacePointsIt != discretization.surfaceIdToPointIndicesMap.end())
    {
        for (size_t idx : surfacePointsIt->second)
        {
            pointIndices.insert(idx);
        }
    }

    return std::vector<size_t>(pointIndices.begin(), pointIndices.end());
}

std::vector<ConstrainedSubfacet3D> FacetTriangulationManager::getAllSubfacets() const
{
    std::vector<ConstrainedSubfacet3D> allSubfacets;

    for (const auto& [surfaceId, facetTriang] : facetTriangulations_)
    {
        auto subfacets = facetTriang->getSubfacets();
        allSubfacets.insert(allSubfacets.end(), subfacets.begin(), subfacets.end());
    }

    return allSubfacets;
}

std::vector<ConstrainedSubfacet3D> FacetTriangulationManager::getSubfacetsForSurface(
    const std::string& surfaceId) const
{
    auto it = facetTriangulations_.find(surfaceId);
    if (it != facetTriangulations_.end())
    {
        return it->second->getSubfacets();
    }
    return {};
}

FacetTriangulation* FacetTriangulationManager::getFacetTriangulation(const std::string& surfaceId)
{
    auto it = facetTriangulations_.find(surfaceId);
    if (it != facetTriangulations_.end())
    {
        return it->second.get();
    }
    return nullptr;
}

const FacetTriangulation* FacetTriangulationManager::getFacetTriangulation(
    const std::string& surfaceId) const
{
    auto it = facetTriangulations_.find(surfaceId);
    if (it != facetTriangulations_.end())
    {
        return it->second.get();
    }
    return nullptr;
}

bool FacetTriangulationManager::insertVertexOnSurface(
    size_t node3DId,
    const Point3D& point,
    const std::string& surfaceId)
{
    auto* facetTriang = getFacetTriangulation(surfaceId);
    if (!facetTriang)
    {
        spdlog::error("FacetTriangulationManager: No triangulation for surface {}", surfaceId);
        return false;
    }

    auto* surface = geometry_->getSurface(surfaceId);
    if (!surface)
    {
        spdlog::error("FacetTriangulationManager: No geometry for surface {}", surfaceId);
        return false;
    }

    // Project 3D point to (u,v) parametric coordinates
    Point2D uvCoord = surface->projectPoint(point);

    return facetTriang->insertVertex(node3DId, uvCoord);
}

} // namespace Meshing
