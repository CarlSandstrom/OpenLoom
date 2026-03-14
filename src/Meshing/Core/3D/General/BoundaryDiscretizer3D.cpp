#include "Meshing/Core/3D/General/BoundaryDiscretizer3D.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ICorner3D.h"
#include "Geometry/3D/Base/IEdge3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Topology/Topology3D.h"
#include <algorithm>
#include <cmath>

namespace Meshing
{

namespace
{

double angleBetweenTangents(const std::array<double, 3>& a, const std::array<double, 3>& b)
{
    double dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    dot = std::clamp(dot, -1.0, 1.0);
    return std::acos(dot);
}

} // namespace

BoundaryDiscretizer3D::BoundaryDiscretizer3D(const Geometry3D::GeometryCollection3D& geometry,
                                             const Topology3D::Topology3D& topology,
                                             const Geometry3D::DiscretizationSettings3D& settings) :
    geometry_(&geometry),
    topology_(&topology),
    settings_(settings),
    result_(std::make_unique<DiscretizationResult3D>())
{
}

void BoundaryDiscretizer3D::discretize()
{
    result_ = std::make_unique<DiscretizationResult3D>();

    // Step 1: Sample corner points
    for (const auto& cornerId : topology_->getAllCornerIds())
    {
        const auto* corner = geometry_->getCorner(cornerId);
        if (!corner)
            continue;

        size_t pointIndex = result_->points.size();
        result_->points.push_back(corner->getPoint());
        result_->edgeParameters.push_back({});
        result_->geometryIds.push_back({});
        result_->cornerIdToPointIndexMap[cornerId] = pointIndex;
    }

    // Step 2: Sample edge interior points (excluding endpoints)
    const auto maxAngle = settings_.getMaxAngleBetweenSegments();
    const auto numSegments = settings_.getNumSegmentsPerEdge();

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

        if (maxAngle.has_value())
        {
            // Angle-based: walk 1000 uniform steps; insert a point whenever the
            // accumulated tangent-angle change since the last inserted point
            // meets or exceeds maxAngle. Straight edges produce no interior points.
            constexpr size_t NUM_STEPS = 1000;
            auto prevTangent = edge->getTangent(tMin);
            double accumulated = 0.0;

            for (size_t i = 1; i <= NUM_STEPS; ++i)
            {
                double t = tMin + static_cast<double>(i) * (tMax - tMin) / static_cast<double>(NUM_STEPS);
                auto nextTangent = edge->getTangent(t);
                accumulated += angleBetweenTangents(prevTangent, nextTangent);
                prevTangent = nextTangent;

                if (accumulated >= maxAngle.value())
                {
                    size_t pointIndex = result_->points.size();
                    result_->points.push_back(edge->getPoint(t));
                    result_->edgeParameters.push_back({t});
                    result_->geometryIds.push_back({edgeId});
                    edgePointIndices.push_back(pointIndex);
                    accumulated = 0.0;
                }

                if (t >= tMax)
                    break;
            }
        }
        else if (numSegments.has_value() && numSegments.value() > 1)
        {
            // Fixed-count: divide the edge into numSegments uniform segments.
            const size_t n = numSegments.value();
            for (size_t i = 1; i < n; ++i)
            {
                double t = tMin + (tMax - tMin) * static_cast<double>(i) / static_cast<double>(n);
                Point3D point = edge->getPoint(t);

                size_t pointIndex = result_->points.size();
                result_->points.push_back(point);
                result_->edgeParameters.push_back({t});
                result_->geometryIds.push_back({edgeId});
                edgePointIndices.push_back(pointIndex);
            }
        }
        // else: no interior points — edge represented by endpoints only.

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

}

const DiscretizationResult3D& BoundaryDiscretizer3D::getDiscretizationResult() const
{
    return *result_;
}

std::unique_ptr<DiscretizationResult3D> BoundaryDiscretizer3D::releaseDiscretizationResult()
{
    return std::move(result_);
}

} // namespace Meshing
