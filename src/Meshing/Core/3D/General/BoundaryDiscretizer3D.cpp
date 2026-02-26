#include "Meshing/Core/3D/General/BoundaryDiscretizer3D.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ICorner3D.h"
#include "Geometry/3D/Base/IEdge3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Topology/Topology3D.h"
#include <algorithm>

namespace Meshing
{

BoundaryDiscretizer3D::BoundaryDiscretizer3D(const Geometry3D::GeometryCollection3D& geometry,
                                             const Topology3D::Topology3D& topology,
                                             const Geometry3D::DiscretizationSettings3D& settings,
                                             const EdgeTwinTable& twinTable) :
    geometry_(&geometry),
    topology_(&topology),
    settings_(settings),
    twinTable_(twinTable),
    result_(std::make_unique<DiscretizationResult3D>()),
    twinManager_(std::make_unique<TwinManager>())
{
}

void BoundaryDiscretizer3D::discretize()
{
    result_ = std::make_unique<DiscretizationResult3D>();
    twinManager_ = std::make_unique<TwinManager>();

    // Step 1: Sample corner points
    for (const auto& cornerId : topology_->getAllCornerIds())
    {
        const auto* corner = geometry_->getCorner(cornerId);
        if (!corner)
            continue;

        size_t pointIndex = result_->points.size();
        result_->points.push_back(corner->getPoint());
        result_->edgeParameters.push_back({});
        result_->geometryIds.push_back({cornerId});
        result_->cornerIdToPointIndexMap[cornerId] = pointIndex;
    }

    // Step 2: Sample edge interior points (excluding endpoints)
    size_t numSegments = settings_.getNumSegmentsPerEdge();

    for (const auto& edgeId : topology_->getAllEdgeIds())
    {
        // Seam twin edges have no 3D geometry; they are handled in Step 2b.
        if (topology_->getSeamCollection().isSeamTwin(edgeId))
            continue;

        const auto& topoEdge = topology_->getEdge(edgeId);
        const auto* edge = geometry_->getEdge(edgeId);

        if (!edge)
            continue;

        auto [tMin, tMax] = edge->getParameterBounds();

        size_t startIdx = result_->cornerIdToPointIndexMap.at(topoEdge.getStartCornerId());
        size_t endIdx = result_->cornerIdToPointIndexMap.at(topoEdge.getEndCornerId());

        result_->edgeParameters[startIdx].push_back(tMin);
        result_->geometryIds[startIdx].push_back(edgeId);
        result_->edgeParameters[endIdx].push_back(tMax);
        result_->geometryIds[endIdx].push_back(edgeId);

        std::vector<size_t> edgePointIndices;
        edgePointIndices.push_back(startIdx);

        for (size_t i = 1; i < numSegments; ++i)
        {
            double t = tMin + (tMax - tMin) * static_cast<double>(i) / static_cast<double>(numSegments);
            Point3D point = edge->getPoint(t);

            size_t pointIndex = result_->points.size();
            result_->points.push_back(point);
            result_->edgeParameters.push_back({t});
            result_->geometryIds.push_back({edgeId});
            edgePointIndices.push_back(pointIndex);
        }

        edgePointIndices.push_back(endIdx);
        result_->edgeIdToPointIndicesMap[edgeId] = edgePointIndices;
    }

    // Step 2b: Populate seam twin edge point sequences (reversed copy of original seam).
    // Seam twin edges have no 3D geometry entry; their point sequence is the reverse of
    // the original seam edge, representing the same curve at U + uPeriod in UV space.
    const auto& seams = topology_->getSeamCollection();
    for (const auto& twinId : seams.getSeamTwinEdgeIds())
    {
        const std::string& originalId = seams.getOriginalEdgeId(twinId);
        auto origIt = result_->edgeIdToPointIndicesMap.find(originalId);
        if (origIt == result_->edgeIdToPointIndicesMap.end())
            continue;

        auto reversed = origIt->second;
        std::reverse(reversed.begin(), reversed.end());
        result_->edgeIdToPointIndicesMap[twinId] = std::move(reversed);
    }

    // Step 3: Sample surface interior points
    size_t surfaceSamples = settings_.getNumSamplesPerSurfaceDirection();

    for (const auto& surfaceId : topology_->getAllSurfaceIds())
    {
        const auto* surface = geometry_->getSurface(surfaceId);

        if (!surface)
            continue;

        auto bounds = surface->getParameterBounds();
        double uMin = bounds.getUMin();
        double uMax = bounds.getUMax();
        double vMin = bounds.getVMin();
        double vMax = bounds.getVMax();

        std::vector<size_t> surfacePointIndices;

        for (size_t i = 1; i < surfaceSamples; ++i)
        {
            double u = uMin + (uMax - uMin) * static_cast<double>(i) / static_cast<double>(surfaceSamples);
            for (size_t j = 1; j < surfaceSamples; ++j)
            {
                double v = vMin + (vMax - vMin) * static_cast<double>(j) / static_cast<double>(surfaceSamples);
                Point3D point = surface->getPoint(u, v);

                size_t pointIndex = result_->points.size();
                result_->points.push_back(point);
                result_->edgeParameters.push_back({});
                result_->geometryIds.push_back({surfaceId});
                surfacePointIndices.push_back(pointIndex);
            }
        }

        result_->surfaceIdToPointIndicesMap[surfaceId] = surfacePointIndices;
    }

    // Step 4: Build twin manager from shared-edge segment pairs
    for (const auto& [edgeId, entries] : twinTable_)
    {
        if (entries.size() != 2)
            continue;

        if (!result_->edgeIdToPointIndicesMap.contains(edgeId))
            continue;

        const auto& edgePoints = result_->edgeIdToPointIndicesMap.at(edgeId);
        if (edgePoints.size() < 2)
            continue;

        const size_t numberOfSegments = edgePoints.size() - 1;

        for (size_t i = 0; i < numberOfSegments; ++i)
        {
            size_t s0From, s0To;
            if (entries[0].orientation == TwinOrientation::Same)
            {
                s0From = edgePoints[i];
                s0To = edgePoints[i + 1];
            }
            else
            {
                s0From = edgePoints[numberOfSegments - i];
                s0To = edgePoints[numberOfSegments - i - 1];
            }

            size_t s1From, s1To;
            if (entries[1].orientation == TwinOrientation::Same)
            {
                s1From = edgePoints[i];
                s1To = edgePoints[i + 1];
            }
            else
            {
                s1From = edgePoints[numberOfSegments - i];
                s1To = edgePoints[numberOfSegments - i - 1];
            }

            twinManager_->registerTwin(s0From, s0To, s1From, s1To);
        }
    }
}

const DiscretizationResult3D& BoundaryDiscretizer3D::getDiscretizationResult() const
{
    return *result_;
}

std::unique_ptr<DiscretizationResult3D> BoundaryDiscretizer3D::releaseDiscretizationResult()
{
    return std::move(result_);
}

const TwinManager& BoundaryDiscretizer3D::getTwinManager() const
{
    return *twinManager_;
}

std::unique_ptr<TwinManager> BoundaryDiscretizer3D::releaseTwinManager()
{
    return std::move(twinManager_);
}

} // namespace Meshing
