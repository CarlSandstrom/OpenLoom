#include "Meshing/Core/3D/General/FacetDiscretization2DBuilder.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Topology/SeamCollection.h"
#include "Topology/Surface3D.h"

namespace Meshing
{

FacetDiscretization2DBuilder::FacetDiscretization2DBuilder(const Geometry3D::ISurface3D& surface,
                                                           const Topology3D::Surface3D& topoSurface,
                                                           const Topology3D::SeamCollection& seams,
                                                           const DiscretizationResult3D& discretization3D,
                                                           const std::vector<size_t>& globalPointIndices,
                                                           std::function<size_t(size_t)> pointIdxToNode3DId,
                                                           const Geometry3D::GeometryCollection3D& geometry) :
    surface_(surface),
    topoSurface_(topoSurface),
    seams_(seams),
    discretization3D_(discretization3D),
    globalPointIndices_(globalPointIndices),
    pointIdxToNode3DId_(std::move(pointIdxToNode3DId)),
    geometry_(geometry)
{
}

void FacetDiscretization2DBuilder::build()
{
    buildGlobalToLocalMap();
    buildPoints();
    buildCornerMap();
    buildEdgeMaps();
}

void FacetDiscretization2DBuilder::buildGlobalToLocalMap()
{
    for (size_t localIndex = 0; localIndex < globalPointIndices_.size(); ++localIndex)
    {
        globalToLocal_[globalPointIndices_[localIndex]] = localIndex;
    }
}

void FacetDiscretization2DBuilder::buildPoints()
{
    discretization2D_.points.reserve(globalPointIndices_.size());
    discretization2D_.tParameters.reserve(globalPointIndices_.size());
    discretization2D_.geometryIds.reserve(globalPointIndices_.size());
    localIndexToNode3DId_.reserve(globalPointIndices_.size());

    for (size_t localIndex = 0; localIndex < globalPointIndices_.size(); ++localIndex)
    {
        size_t globalPointIndex = globalPointIndices_[localIndex];
        discretization2D_.points.push_back(surface_.projectPoint(discretization3D_.points[globalPointIndex]));

        // Normalise 3D edge parameters to [0,1] for use with LinearEdge2D.
        const auto& edgeParameters = discretization3D_.edgeParameters[globalPointIndex];
        const auto& geometryIds = discretization3D_.geometryIds[globalPointIndex];
        std::vector<double> normalizedParams;
        normalizedParams.reserve(edgeParameters.size());
        for (size_t k = 0; k < edgeParameters.size(); ++k)
        {
            double t = edgeParameters[k];
            if (k < geometryIds.size())
            {
                const auto* edge3D = geometry_.getEdge(geometryIds[k]);
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
        discretization2D_.tParameters.push_back(std::move(normalizedParams));
        discretization2D_.geometryIds.push_back(discretization3D_.geometryIds[globalPointIndex]);
        localIndexToNode3DId_.push_back(pointIdxToNode3DId_(globalPointIndex));
    }
}

void FacetDiscretization2DBuilder::buildCornerMap()
{
    for (const auto& cornerId : topoSurface_.getCornerIds())
    {
        auto it = discretization3D_.cornerIdToPointIndexMap.find(cornerId);
        if (it != discretization3D_.cornerIdToPointIndexMap.end())
        {
            auto localIt = globalToLocal_.find(it->second);
            if (localIt != globalToLocal_.end())
            {
                discretization2D_.cornerIdToPointIndexMap[cornerId] = localIt->second;
            }
        }
    }
}

void FacetDiscretization2DBuilder::buildEdgeMaps()
{
    auto realCornerToShiftedLocal = processSeamEdges();
    processNonSeamEdges(realCornerToShiftedLocal);
}

std::map<size_t, size_t> FacetDiscretization2DBuilder::processSeamEdges()
{
    // Phase 1: Duplicate seam-twin edge points at U + uPeriod so the periodic
    // boundary closes correctly in UV space. Returns realCornerToShiftedLocal,
    // which phase 2 uses to close any circular (start == end) edges.
    std::map<size_t, size_t> realCornerToShiftedLocal;

    std::vector<std::string> seamTwinEdgeIds;
    for (const auto& edgeId : topoSurface_.getBoundaryEdgeIds())
    {
        if (seams_.isSeamTwin(edgeId))
            seamTwinEdgeIds.push_back(edgeId);
    }

    if (seamTwinEdgeIds.empty())
        return realCornerToShiftedLocal;

    const auto bounds = surface_.getParameterBounds();
    const double uPeriod = bounds.getUMax() - bounds.getUMin();

    for (const auto& seamId : seamTwinEdgeIds)
    {
        auto it = discretization3D_.edgeIdToPointIndicesMap.find(seamId);
        if (it == discretization3D_.edgeIdToPointIndicesMap.end())
            continue;

        std::vector<size_t> seamLocalIndices;
        seamLocalIndices.reserve(it->second.size());

        for (size_t globalIdx : it->second)
        {
            size_t newLocalIdx = discretization2D_.points.size();

            // Project to UV and shift to the far side of the seam.
            auto uv = surface_.projectPoint(discretization3D_.points[globalIdx]);
            discretization2D_.points.push_back(Point2D(uv.x() + uPeriod, uv.y()));

            // Copy metadata from the original local entry, replacing the original
            // seam edge ID with the seam twin ID so that findCommonGeometryId /
            // splitConstrainedSegment use the shifted UV edge.
            auto origIt = globalToLocal_.find(globalIdx);
            if (origIt != globalToLocal_.end())
            {
                size_t origLocalIdx = origIt->second;
                discretization2D_.tParameters.push_back(discretization2D_.tParameters[origLocalIdx]);

                const std::string& originalSeamId = seams_.getOriginalEdgeId(seamId);
                auto geoIdsCopy = discretization2D_.geometryIds[origLocalIdx];
                for (auto& geoId : geoIdsCopy)
                {
                    if (geoId == originalSeamId)
                        geoId = seamId;
                }
                discretization2D_.geometryIds.push_back(std::move(geoIdsCopy));
            }
            else
            {
                discretization2D_.tParameters.push_back({});
                discretization2D_.geometryIds.push_back({});
            }

            localIndexToNode3DId_.push_back(pointIdxToNode3DId_(globalIdx));
            realCornerToShiftedLocal[globalIdx] = newLocalIdx;
            seamLocalIndices.push_back(newLocalIdx);
        }

        discretization2D_.edgeIdToPointIndicesMap[seamId] = std::move(seamLocalIndices);
    }

    return realCornerToShiftedLocal;
}

void FacetDiscretization2DBuilder::processNonSeamEdges(
    const std::map<size_t, size_t>& realCornerToShiftedLocal)
{
    // Phase 2: Map non-seam edges to local indices. Closed-circle edges (start
    // and end share the same 3D global index) redirect their endpoint to the
    // seam-shifted local index so the boundary closes at U + uPeriod.
    for (const auto& edgeId : topoSurface_.getBoundaryEdgeIds())
    {
        if (seams_.isSeamTwin(edgeId))
            continue;

        auto it = discretization3D_.edgeIdToPointIndicesMap.find(edgeId);
        if (it == discretization3D_.edgeIdToPointIndicesMap.end())
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

            bool isClosedCircleEndpoint = (i == globalSeq.size() - 1) &&
                                          (globalIdx == firstGlobal) &&
                                          realCornerToShiftedLocal.count(globalIdx);
            if (isClosedCircleEndpoint)
            {
                localEdgeIndices.push_back(realCornerToShiftedLocal.at(globalIdx));
            }
            else
            {
                auto localIt = globalToLocal_.find(globalIdx);
                if (localIt != globalToLocal_.end())
                {
                    localEdgeIndices.push_back(localIt->second);
                }
            }
        }

        discretization2D_.edgeIdToPointIndicesMap[edgeId] = std::move(localEdgeIndices);
    }
}

} // namespace Meshing
