#include "Meshing/Core/3D/General/FacetTriangulationManager.h"
#include "Common/TwinManager.h"
#include "Common/Types.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Core/3D/General/EdgeTwinTable.h"
#include "Meshing/Core/3D/General/FacetDiscretization2DBuilder.h"
#include "Meshing/Core/3D/General/TwinTableGenerator.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Topology/SeamCollection.h"
#include "Topology/Surface3D.h"
#include "Topology/Topology3D.h"
#include "spdlog/spdlog.h"
#include <map>
#include <set>

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

FacetTriangulationManager FacetTriangulationManager::createForSurfaceMesher(const Geometry3D::GeometryCollection3D& geometry,
                                                                            const Topology3D::Topology3D& topology,
                                                                            const DiscretizationResult3D& discretization)
{
    FacetTriangulationManager manager(geometry, topology);
    manager.initializeForSurfaceMesher(discretization);
    return manager;
}

FacetTriangulationManager FacetTriangulationManager::createForVolumeMesher(const Geometry3D::GeometryCollection3D& geometry,
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
// Twin manager
// -----------------------------------------------------------------------

std::unique_ptr<TwinManager> FacetTriangulationManager::releaseTwinManager()
{
    return std::move(twinManager_);
}

// -----------------------------------------------------------------------
// Private initialisation helpers
// -----------------------------------------------------------------------

void FacetTriangulationManager::initializeForSurfaceMesher(const DiscretizationResult3D& discretization)
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

        auto facetTriangulator = std::make_unique<FacetTriangulation>(*surface, topoSurface, *topology_, *geometry_);

        // In the surface-mesher path, point index == 3D node ID — identity mapping.
        // Use boundary-only points so the triangulator starts clean and refinement
        // adds interior vertices as needed.
        std::vector<size_t> globalPtIndices = collectBoundaryPointIndices(surfaceId, discretization);
        FacetDiscretization2DBuilder builder(*surface, topoSurface, topology_->getSeamCollection(),
                                             discretization, globalPtIndices,
                                             [](size_t ptIdx) { return ptIdx; },
                                             *geometry_);
        builder.build();

        auto discretization2D = builder.takeDiscretization2D();
        auto localIndexToNode3DId = builder.takeLocalIndexToNode3DId();
        size_t pointCount = discretization2D.points.size();
        facetTriangulator->initialize(std::move(discretization2D), std::move(localIndexToNode3DId));

        spdlog::debug("FacetTriangulationManager: Created triangulation for surface {} with {} points",
                      surfaceId, pointCount);

        facetTriangulations_[surfaceId] = std::move(facetTriangulator);
    }

    spdlog::info("FacetTriangulationManager: Initialized {} facet triangulations",
                 facetTriangulations_.size());

    buildTwinManager();
}

void FacetTriangulationManager::initializeForVolumeMesher(const DiscretizationResult3D& discretization,
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
        FacetDiscretization2DBuilder builder(*surface, topoSurface, topology_->getSeamCollection(),
                                             discretization, globalPtIndices,
                                             [&pointIndexToNodeIdMap](size_t ptIdx)
                                             {
                                                 auto it = pointIndexToNodeIdMap.find(ptIdx);
                                                 return (it != pointIndexToNodeIdMap.end()) ? it->second : ptIdx;
                                             },
                                             *geometry_);
        builder.build();

        auto discretization2D = builder.takeDiscretization2D();
        auto localIndexToNode3DId = builder.takeLocalIndexToNode3DId();
        size_t pointCount = discretization2D.points.size();
        facetTriang->initialize(std::move(discretization2D), std::move(localIndexToNode3DId));

        spdlog::debug("FacetTriangulationManager: Created triangulation for surface {} with {} points",
                      surfaceId, pointCount);

        facetTriangulations_[surfaceId] = std::move(facetTriang);
    }

    spdlog::info("FacetTriangulationManager: Initialized {} facet triangulations",
                 facetTriangulations_.size());
}

void FacetTriangulationManager::buildTwinManager()
{
    twinManager_ = std::make_unique<TwinManager>();

    // -----------------------------------------------------------------------
    // Cross-surface twins: edges shared between two different surfaces.
    // The EdgeTwinTable gives us the orientation each surface uses when
    // traversing the shared edge (Same = canonical direction, Reversed = opposite).
    // -----------------------------------------------------------------------
    const EdgeTwinTable twinTable = TwinTableGenerator::generate(*topology_);

    for (const auto& [edgeId, entries] : twinTable)
    {
        if (entries.size() != 2)
            continue;

        const std::string& surfaceId0 = entries[0].surfaceId;
        const std::string& surfaceId1 = entries[1].surfaceId;

        auto* facet0 = getFacetTriangulation(surfaceId0);
        auto* facet1 = getFacetTriangulation(surfaceId1);
        if (!facet0 || !facet1)
            continue;

        const auto* seq0 = facet0->getEdgeNodeSequence(edgeId);
        const auto* seq1 = facet1->getEdgeNodeSequence(edgeId);
        if (!seq0 || !seq1 || seq0->size() < 2 || seq1->size() != seq0->size())
        {
            spdlog::warn("FacetTriangulationManager: Cannot build twin for edge {} "
                         "(seq sizes: {}/{})",
                         edgeId,
                         seq0 ? seq0->size() : 0,
                         seq1 ? seq1->size() : 0);
            continue;
        }

        // Both sequences are in canonical (start→end) order from BoundaryDiscretizer3D,
        // so seq0[i] and seq1[i] represent the same 3D point. Pair by index directly.
        const size_t numberOfSegments = seq0->size() - 1;
        for (size_t i = 0; i < numberOfSegments; ++i)
        {
            twinManager_->registerTwin(surfaceId0, (*seq0)[i], (*seq0)[i + 1],
                                       surfaceId1, (*seq1)[i], (*seq1)[i + 1]);
        }

        spdlog::debug("FacetTriangulationManager: Registered {} cross-surface twin pairs for edge {}",
                      numberOfSegments, edgeId);
    }

    // -----------------------------------------------------------------------
    // Seam twins: both sides of a periodic seam on the same surface.
    // The original seam sequence origSeq and the twin sequence twinSeq are
    // related by: origSeq[i] ↔ twinSeq[N-i] (same 3D point, different UV).
    // So segment i on the original side pairs with twinSeq[N-i]→twinSeq[N-i-1].
    // -----------------------------------------------------------------------
    const auto& seams = topology_->getSeamCollection();

    for (const auto& twinEdgeId : seams.getSeamTwinEdgeIds())
    {
        const std::string& origEdgeId = seams.getOriginalEdgeId(twinEdgeId);

        for (const auto& surfaceId : topology_->getAllSurfaceIds())
        {
            auto* facet = getFacetTriangulation(surfaceId);
            if (!facet)
                continue;

            const auto* origSeq = facet->getEdgeNodeSequence(origEdgeId);
            const auto* twinSeq = facet->getEdgeNodeSequence(twinEdgeId);
            if (!origSeq || !twinSeq || origSeq->size() < 2 || twinSeq->size() != origSeq->size())
                continue;

            const size_t numberOfSegments = origSeq->size() - 1;
            for (size_t i = 0; i < numberOfSegments; ++i)
            {
                twinManager_->registerTwin(
                    surfaceId, (*origSeq)[i], (*origSeq)[i + 1],
                    surfaceId, (*twinSeq)[numberOfSegments - i], (*twinSeq)[numberOfSegments - i - 1]);
            }

            spdlog::debug("FacetTriangulationManager: Registered {} seam twin pairs "
                          "for surface {} (orig={}, twin={})",
                          numberOfSegments, surfaceId, origEdgeId, twinEdgeId);
            break; // Each seam pair belongs to exactly one surface
        }
    }
}

// -----------------------------------------------------------------------
// Private helpers
// -----------------------------------------------------------------------

std::vector<size_t> FacetTriangulationManager::collectBoundaryPointIndices(const std::string& surfaceId,
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

    // Add edge points (from boundary edges, excludes seam twins — they share 3D points)
    for (const auto& edgeId : topoSurface.getBoundaryEdgeIds())
    {
        if (topology_->getSeamCollection().isSeamTwin(edgeId))
            continue;

        auto it = discretization.edgeIdToPointIndicesMap.find(edgeId);
        if (it != discretization.edgeIdToPointIndicesMap.end())
        {
            for (size_t index : it->second)
            {
                pointIndices.insert(index);
            }
        }
    }

    return std::vector<size_t>(pointIndices.begin(), pointIndices.end());
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
            for (size_t index : it->second)
            {
                pointIndices.insert(index);
            }
        }
    }

    // Add interior surface points
    auto surfacePointsIt = discretization.surfaceIdToPointIndicesMap.find(surfaceId);
    if (surfacePointsIt != discretization.surfaceIdToPointIndicesMap.end())
    {
        for (size_t index : surfacePointsIt->second)
        {
            pointIndices.insert(index);
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
