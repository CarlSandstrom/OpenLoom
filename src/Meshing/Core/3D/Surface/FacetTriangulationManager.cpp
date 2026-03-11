#include "Meshing/Core/3D/Surface/FacetTriangulationManager.h"
#include "Common/Types.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Core/2D/DiscretizationResult2D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Topology/SeamCollection.h"
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
// Also returns localIndexToNode3DId mapping each local index → 3D node ID.
std::pair<Meshing::DiscretizationResult2D, std::vector<size_t>> buildFaceDiscretization2D(const Geometry3D::ISurface3D& surface,
                                                                                          const Topology3D::Surface3D& topoSurface,
                                                                                          const Topology3D::SeamCollection& seams,
                                                                                          const Meshing::DiscretizationResult3D& discretization3D,
                                                                                          const std::vector<size_t>& globalPointIndices,
                                                                                          const std::function<size_t(size_t)>& pointIdxToNode3DId,
                                                                                          const Geometry3D::GeometryCollection3D& geometry)
{
    Meshing::DiscretizationResult2D discretization2D;
    std::vector<size_t> localIndexToNode3DId;

    // Build global → local index lookup (globalPointIndices is already sorted)
    std::map<size_t, size_t> globalToLocal;
    for (size_t localIndex = 0; localIndex < globalPointIndices.size(); ++localIndex)
    {
        globalToLocal[globalPointIndices[localIndex]] = localIndex;
    }

    // Build points, tParameters, geometryIds, and localIndexToNode3DId in lockstep
    discretization2D.points.reserve(globalPointIndices.size());
    discretization2D.tParameters.reserve(globalPointIndices.size());
    discretization2D.geometryIds.reserve(globalPointIndices.size());
    localIndexToNode3DId.reserve(globalPointIndices.size());

    for (size_t localIndex = 0; localIndex < globalPointIndices.size(); ++localIndex)
    {
        size_t globalPointIndex = globalPointIndices[localIndex];
        discretization2D.points.push_back(surface.projectPoint(discretization3D.points[globalPointIndex]));

        // Normalize 3D edge parameters to [0,1] for use with LinearEdge2D.
        const auto& edgeParameters = discretization3D.edgeParameters[globalPointIndex];
        const auto& geometryIds = discretization3D.geometryIds[globalPointIndex];
        std::vector<double> normalizedParams;
        normalizedParams.reserve(edgeParameters.size());
        for (size_t k = 0; k < edgeParameters.size(); ++k)
        {
            double t = edgeParameters[k];
            if (k < geometryIds.size())
            {
                const auto* edge3D = geometry.getEdge(geometryIds[k]);
                if (edge3D)
                {
                    auto [tMin, tMax] = edge3D->getParameterBounds();
                    if (tMax > tMin)
                    {
                        t = (t - tMin) / (tMax - tMin);
                    }
                }
            }
            normalizedParams.push_back(t);
        }
        discretization2D.tParameters.push_back(normalizedParams);
        discretization2D.geometryIds.push_back(discretization3D.geometryIds[globalPointIndex]);
        localIndexToNode3DId.push_back(pointIdxToNode3DId(globalPointIndex));
    }

    // Build cornerIdToPointIndexMap: translate global corner index → local
    for (const auto& cornerId : topoSurface.getCornerIds())
    {
        auto it = discretization3D.cornerIdToPointIndexMap.find(cornerId);
        if (it != discretization3D.cornerIdToPointIndexMap.end())
        {
            auto localIt = globalToLocal.find(it->second);
            if (localIt != globalToLocal.end())
            {
                discretization2D.cornerIdToPointIndexMap[cornerId] = localIt->second;
            }
        }
    }

    // Partition boundary edges into seam twins and regular edges.
    std::vector<std::string> seamTwinEdgeIds;
    std::vector<std::string> nonSeamEdgeIds;
    for (const auto& edgeId : topoSurface.getBoundaryEdgeIds())
    {
        if (seams.isSeamTwin(edgeId))
            seamTwinEdgeIds.push_back(edgeId);
        else
            nonSeamEdgeIds.push_back(edgeId);
    }

    // Phase 1: Process seam twin edges FIRST to build realCornerToShiftedLocal.
    // Each global point index in the seam twin sequence gets a UV-shifted copy
    // (at U + uPeriod). Knowing these shifted local indices up front lets phase 2
    // correctly close any periodic (closed-circle) edges.
    std::map<size_t, size_t> realCornerToShiftedLocal;
    if (!seamTwinEdgeIds.empty())
    {
        const auto bounds = surface.getParameterBounds();
        const double uPeriod = bounds.getUMax() - bounds.getUMin();

        for (const auto& seamId : seamTwinEdgeIds)
        {
            auto it = discretization3D.edgeIdToPointIndicesMap.find(seamId);
            if (it == discretization3D.edgeIdToPointIndicesMap.end())
                continue;

            std::vector<size_t> seamLocalIndices;
            seamLocalIndices.reserve(it->second.size());

            for (size_t globalIdx : it->second)
            {
                size_t newLocalIdx = discretization2D.points.size();

                // Project to UV and shift to the far side of the seam
                auto uv = surface.projectPoint(discretization3D.points[globalIdx]);
                discretization2D.points.push_back(Meshing::Point2D(uv.x() + uPeriod, uv.y()));

                // Copy metadata from the original local entry (if available),
                // replacing the original seam edge ID with the seam twin ID so that
                // findCommonGeometryId / splitConstrainedSegment use the shifted UV edge.
                auto origIt = globalToLocal.find(globalIdx);
                if (origIt != globalToLocal.end())
                {
                    size_t origLocalIdx = origIt->second;
                    discretization2D.tParameters.push_back(discretization2D.tParameters[origLocalIdx]);

                    const std::string& originalSeamId = seams.getOriginalEdgeId(seamId);
                    auto geoIdsCopy = discretization2D.geometryIds[origLocalIdx];
                    for (auto& geoId : geoIdsCopy)
                    {
                        if (geoId == originalSeamId)
                            geoId = seamId;
                    }
                    discretization2D.geometryIds.push_back(std::move(geoIdsCopy));
                }
                else
                {
                    discretization2D.tParameters.push_back({});
                    discretization2D.geometryIds.push_back({});
                }

                localIndexToNode3DId.push_back(pointIdxToNode3DId(globalIdx));
                realCornerToShiftedLocal[globalIdx] = newLocalIdx;
                seamLocalIndices.push_back(newLocalIdx);
            }

            discretization2D.edgeIdToPointIndicesMap[seamId] = std::move(seamLocalIndices);
        }
    }

    // Phase 2: Process non-seam edges.
    // Closed-circle edges (start and end share the same 3D global index, e.g. the
    // bottom/top circles of a cylinder) need their final point to reference the
    // seam-shifted local index rather than the original, so the boundary closes
    // at U + uPeriod instead of doubling back to U.
    for (const auto& edgeId : nonSeamEdgeIds)
    {
        auto it = discretization3D.edgeIdToPointIndicesMap.find(edgeId);
        if (it == discretization3D.edgeIdToPointIndicesMap.end())
            continue;

        const auto& globalSeq = it->second;
        if (globalSeq.empty())
            continue;

        const size_t firstGlobal = globalSeq.front();
        std::vector<size_t> localEdgeIndices;
        localEdgeIndices.reserve(globalSeq.size());

        for (size_t i = 0; i < globalSeq.size(); ++i)
        {
            size_t globalIdx = globalSeq[i];

            // Detect the endpoint of a closed circle: last element == first element
            // and a shifted copy exists from phase 1.
            bool isClosedCircleEndpoint = (i == globalSeq.size() - 1) &&
                                          (globalIdx == firstGlobal) &&
                                          realCornerToShiftedLocal.count(globalIdx);
            if (isClosedCircleEndpoint)
            {
                localEdgeIndices.push_back(realCornerToShiftedLocal.at(globalIdx));
            }
            else
            {
                auto localIt = globalToLocal.find(globalIdx);
                if (localIt != globalToLocal.end())
                {
                    localEdgeIndices.push_back(localIt->second);
                }
            }
        }

        discretization2D.edgeIdToPointIndicesMap[edgeId] = std::move(localEdgeIndices);
    }

    return {std::move(discretization2D), std::move(localIndexToNode3DId)};
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
        auto [discretization2D, localIndexToNode3DId] = buildFaceDiscretization2D(*surface, topoSurface, topology_->getSeamCollection(), discretization, globalPtIndices, [](size_t ptIdx)
                                                                                  { return ptIdx; }, *geometry_);

        facetTriangulator->initialize(discretization2D, localIndexToNode3DId);

        spdlog::debug("FacetTriangulationManager: Created triangulation for surface {} with {} points",
                      surfaceId, discretization2D.points.size());

        facetTriangulations_[surfaceId] = std::move(facetTriangulator);
    }

    spdlog::info("FacetTriangulationManager: Initialized {} facet triangulations",
                 facetTriangulations_.size());
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
        auto [disc2D, localIndexToNode3DId] = buildFaceDiscretization2D(
            *surface, topoSurface, topology_->getSeamCollection(), discretization, globalPtIndices,
            [&pointIndexToNodeIdMap](size_t ptIdx)
            {
                auto it = pointIndexToNodeIdMap.find(ptIdx);
                return (it != pointIndexToNodeIdMap.end()) ? it->second : ptIdx;
            },
            *geometry_);

        facetTriang->initialize(disc2D, localIndexToNode3DId);

        spdlog::debug("FacetTriangulationManager: Created triangulation for surface {} with {} points",
                      surfaceId, disc2D.points.size());

        facetTriangulations_[surfaceId] = std::move(facetTriang);
    }

    spdlog::info("FacetTriangulationManager: Initialized {} facet triangulations",
                 facetTriangulations_.size());
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
