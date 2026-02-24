#include "Meshing/Core/3D/Surface/FacetTriangulationManager.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Core/2D/DiscretizationResult2D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Topology/Surface3D.h"
#include "Topology/Topology3D.h"
#include "spdlog/spdlog.h"
#include <functional>
#include <map>
#include <set>

namespace
{

// Build the UV-space DiscretizationResult2D for a single face, using LOCAL indices
// (0..N-1 over the sorted global point indices for this face).
// Also returns localIdxToNode3DId mapping each local index → 3D node ID.
std::pair<Meshing::DiscretizationResult2D, std::vector<size_t>>
buildFaceDiscretization2D(
    const Geometry3D::ISurface3D& surface,
    const Topology3D::Surface3D& topoSurface,
    const Meshing::DiscretizationResult3D& disc3D,
    const std::vector<size_t>& globalPtIndices,
    const std::function<size_t(size_t)>& pointIdxToNode3DId)
{
    Meshing::DiscretizationResult2D disc2D;
    std::vector<size_t> localIdxToNode3DId;

    // Build global → local index lookup (globalPtIndices is already sorted)
    std::map<size_t, size_t> globalToLocal;
    for (size_t localIdx = 0; localIdx < globalPtIndices.size(); ++localIdx)
    {
        globalToLocal[globalPtIndices[localIdx]] = localIdx;
    }

    // Build points, tParameters, geometryIds, and localIdxToNode3DId in lockstep
    disc2D.points.reserve(globalPtIndices.size());
    disc2D.tParameters.reserve(globalPtIndices.size());
    disc2D.geometryIds.reserve(globalPtIndices.size());
    localIdxToNode3DId.reserve(globalPtIndices.size());

    for (size_t localIdx = 0; localIdx < globalPtIndices.size(); ++localIdx)
    {
        size_t globalPtIdx = globalPtIndices[localIdx];
        disc2D.points.push_back(surface.projectPoint(disc3D.points[globalPtIdx]));
        disc2D.tParameters.push_back(disc3D.edgeParameters[globalPtIdx]);
        disc2D.geometryIds.push_back(disc3D.geometryIds[globalPtIdx]);
        localIdxToNode3DId.push_back(pointIdxToNode3DId(globalPtIdx));
    }

    // Build cornerIdToPointIndexMap: translate global corner index → local
    for (const auto& cornerId : topoSurface.getCornerIds())
    {
        auto it = disc3D.cornerIdToPointIndexMap.find(cornerId);
        if (it != disc3D.cornerIdToPointIndexMap.end())
        {
            auto localIt = globalToLocal.find(it->second);
            if (localIt != globalToLocal.end())
            {
                disc2D.cornerIdToPointIndexMap[cornerId] = localIt->second;
            }
        }
    }

    // Build edgeIdToPointIndicesMap: translate global edge point indices → local
    for (const auto& edgeId : topoSurface.getBoundaryEdgeIds())
    {
        auto it = disc3D.edgeIdToPointIndicesMap.find(edgeId);
        if (it != disc3D.edgeIdToPointIndicesMap.end())
        {
            std::vector<size_t> localEdgeIndices;
            localEdgeIndices.reserve(it->second.size());
            for (size_t globalIdx : it->second)
            {
                auto localIt = globalToLocal.find(globalIdx);
                if (localIt != globalToLocal.end())
                {
                    localEdgeIndices.push_back(localIt->second);
                }
            }
            disc2D.edgeIdToPointIndicesMap[edgeId] = std::move(localEdgeIndices);
        }
    }

    return {std::move(disc2D), std::move(localIdxToNode3DId)};
}

} // anonymous namespace

namespace Meshing
{

// -----------------------------------------------------------------------
// Private constructor
// -----------------------------------------------------------------------

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

// -----------------------------------------------------------------------
// Factory methods
// -----------------------------------------------------------------------

FacetTriangulationManager FacetTriangulationManager::createForSurfaceMesher(
    const Geometry3D::GeometryCollection3D& geometry,
    const Topology3D::Topology3D& topology,
    const DiscretizationResult3D& discretization)
{
    FacetTriangulationManager manager(geometry, topology);
    manager.initializeForSurfaceMesher(discretization);
    return manager;
}

FacetTriangulationManager FacetTriangulationManager::createForVolumeMesher(
    const Geometry3D::GeometryCollection3D& geometry,
    const Topology3D::Topology3D& topology,
    const DiscretizationResult3D& discretization,
    const std::map<size_t, size_t>& pointIndexToNodeIdMap,
    const MeshData3D& meshData)
{
    FacetTriangulationManager manager(geometry, topology);
    manager.initializeForVolumeMesher(discretization, pointIndexToNodeIdMap, meshData);
    return manager;
}

// -----------------------------------------------------------------------
// Private initialisation helpers
// -----------------------------------------------------------------------

void FacetTriangulationManager::initializeForSurfaceMesher(
    const DiscretizationResult3D& discretization)
{
    facetTriangulations_.clear();

    for (const auto& surfaceId : topology_->getAllSurfaceIds())
    {
        const auto& topoSurface = topology_->getSurface(surfaceId);
        auto* surface = geometry_->getSurface(surfaceId);

        if (!surface)
        {
            spdlog::warn("FacetTriangulationManager: No geometry for surface {}", surfaceId);
            continue;
        }

        auto facetTriang = std::make_unique<FacetTriangulation>(*surface, topoSurface, *topology_, *geometry_);

        // In the surface-mesher path, point index == 3D node ID — identity mapping.
        std::vector<size_t> globalPtIndices = collectSurfacePointIndices(surfaceId, discretization);
        auto [disc2D, localIdxToNode3DId] = buildFaceDiscretization2D(
            *surface, topoSurface, discretization, globalPtIndices,
            [](size_t ptIdx)
            { return ptIdx; });

        facetTriang->initialize(disc2D, localIdxToNode3DId);

        spdlog::debug("FacetTriangulationManager: Created triangulation for surface {} with {} points",
                      surfaceId, disc2D.points.size());

        facetTriangulations_[surfaceId] = std::move(facetTriang);
    }

    spdlog::info("FacetTriangulationManager: Initialized {} facet triangulations",
                 facetTriangulations_.size());
}

void FacetTriangulationManager::initializeForVolumeMesher(
    const DiscretizationResult3D& discretization,
    const std::map<size_t, size_t>& pointIndexToNodeIdMap,
    const MeshData3D& /* meshData */)
{
    facetTriangulations_.clear();

    for (const auto& surfaceId : topology_->getAllSurfaceIds())
    {
        const auto& topoSurface = topology_->getSurface(surfaceId);
        auto* surface = geometry_->getSurface(surfaceId);

        if (!surface)
        {
            spdlog::warn("FacetTriangulationManager: No geometry for surface {}", surfaceId);
            continue;
        }

        auto facetTriang = std::make_unique<FacetTriangulation>(
            *surface, topoSurface, *topology_, *geometry_);

        std::vector<size_t> globalPtIndices = collectSurfacePointIndices(surfaceId, discretization);
        auto [disc2D, localIdxToNode3DId] = buildFaceDiscretization2D(
            *surface, topoSurface, discretization, globalPtIndices,
            [&pointIndexToNodeIdMap](size_t ptIdx)
            {
                auto it = pointIndexToNodeIdMap.find(ptIdx);
                return (it != pointIndexToNodeIdMap.end()) ? it->second : ptIdx;
            });

        facetTriang->initialize(disc2D, localIdxToNode3DId);

        spdlog::debug("FacetTriangulationManager: Created triangulation for surface {} with {} points",
                      surfaceId, disc2D.points.size());

        facetTriangulations_[surfaceId] = std::move(facetTriang);
    }

    spdlog::info("FacetTriangulationManager: Initialized {} facet triangulations",
                 facetTriangulations_.size());
}

// -----------------------------------------------------------------------
// Private helper
// -----------------------------------------------------------------------

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

// -----------------------------------------------------------------------
// Queries
// -----------------------------------------------------------------------

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

    Point2D uvCoord = surface->projectPoint(point);

    return facetTriang->insertVertex(node3DId, uvCoord);
}

} // namespace Meshing
