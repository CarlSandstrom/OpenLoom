#include "EdgeDiscretizer2D.h"
#include "Common/MathConstants.h"
#include "Geometry/2D/Base/GeometryCollection2D.h"
#include "Geometry/2D/Base/ICorner2D.h"
#include "Geometry/2D/Base/IEdge2D.h"
#include "MeshingContext2D.h"
#include "Topology2D/Topology2D.h"
#include <cmath>

namespace Meshing
{

EdgeDiscretizer2D::EdgeDiscretizer2D(const MeshingContext2D& context,
                                     const Geometry2D::DiscretizationSettings2D& settings) :
    context_(&context),
    settings_(settings)
{
}

namespace
{

Eigen::Vector2d computeEdgeTangentAtParameter(const Geometry2D::IEdge2D& edge, double parameter)
{
    double t1 = parameter - epsilon;
    double t2 = parameter + epsilon;

    // Clamp parameters to valid range
    auto bounds = edge.getParameterBounds();
    t1 = std::max(bounds.first, std::min(bounds.second, t1));
    t2 = std::max(bounds.first, std::min(bounds.second, t2));

    Point2D p1 = edge.getPoint(t1);
    Point2D p2 = edge.getPoint(t2);

    Eigen::Vector2d tangent(p2.x() - p1.x(), p2.y() - p1.y());
    tangent.normalize();

    return tangent;
}

} // namespace

DiscretizationResult2D EdgeDiscretizer2D::discretize() const
{
    DiscretizationResult2D result;
    const auto& geometry = context_->getGeometry();
    const auto& topology = context_->getTopology();

    // Extract corner points first
    for (const auto& cornerId : geometry.getAllCornerIds())
    {
        const auto* corner = geometry.getCorner(cornerId);
        result.points.push_back(corner->getPoint());
        result.tParameters.push_back({});
        result.geometryIds.push_back({});
        result.cornerIdToPointIndexMap[cornerId] = result.points.size() - 1;
    }

    // Add intermediate points along edges based on discretization settings
    auto numSegments = settings_.getNumSegmentsPerEdge();
    auto largestAngle = settings_.getMaxAngleBetweenSegments();

    for (const auto& edgeId : topology.getAllEdgeIds())
    {
        const auto& edgeTopology = topology.getEdge(edgeId);
        const auto* edgeGeometry = geometry.getEdge(edgeId);
        auto parameterBounds = edgeGeometry->getParameterBounds();

        // Get start and end corner point indices
        size_t startCornerIdx = result.cornerIdToPointIndexMap.at(edgeTopology.getStartCornerId());
        size_t endCornerIdx = result.cornerIdToPointIndexMap.at(edgeTopology.getEndCornerId());

        // Add t values and geometry IDs for corners on this edge
        result.tParameters[startCornerIdx].push_back(parameterBounds.first);
        result.geometryIds[startCornerIdx].push_back(edgeId);
        result.tParameters[endCornerIdx].push_back(parameterBounds.second);
        result.geometryIds[endCornerIdx].push_back(edgeId);

        // Initialize the edge point indices list with the start corner
        std::vector<size_t> edgePointIndices;
        edgePointIndices.push_back(startCornerIdx);

        // Add intermediate points if discretization is enabled
        if (largestAngle.has_value())
        {
            Eigen::Vector2d prevTangent = computeEdgeTangentAtParameter(*edgeGeometry, parameterBounds.first);
            Point2D startCornerPoint = result.points[startCornerIdx];
            Point2D endCornerPoint = result.points[endCornerIdx];

            for (size_t i = 1; i <= 1000; ++i) // Limit to 1000 segments to avoid infinite loops
            {
                double t = parameterBounds.first + i * (parameterBounds.second - parameterBounds.first) / 1000.0;
                auto nextTangent = computeEdgeTangentAtParameter(*edgeGeometry, t);

                double angle = std::atan2(nextTangent.y(), nextTangent.x()) - std::atan2(prevTangent.y(), prevTangent.x());
                angle = std::fabs(angle);
                if (angle > pi)
                    angle = 2 * pi - angle;

                if (angle >= largestAngle.value())
                {
                    Point2D point = edgeGeometry->getPoint(t);

                    // Skip if too close to start or end corners to avoid duplicate points
                    double distToStart = std::sqrt(std::pow(point.x() - startCornerPoint.x(), 2) +
                                                   std::pow(point.y() - startCornerPoint.y(), 2));
                    double distToEnd = std::sqrt(std::pow(point.x() - endCornerPoint.x(), 2) +
                                                 std::pow(point.y() - endCornerPoint.y(), 2));
                    if (distToStart < 1e-6 || distToEnd < 1e-6)
                    {
                        prevTangent = nextTangent;
                        continue;
                    }

                    size_t pointIndex = result.points.size();
                    result.points.push_back(point);
                    result.tParameters.push_back({t});
                    result.geometryIds.push_back({edgeId});
                    edgePointIndices.push_back(pointIndex);
                    prevTangent = nextTangent;
                }

                if (t >= parameterBounds.second - epsilon)
                    break;
            }
        }
        else if (numSegments.has_value() && numSegments.value() > 1)
        {
            for (size_t i = 1; i < numSegments.value(); ++i)
            {
                double t = parameterBounds.first + i * (parameterBounds.second - parameterBounds.first) / numSegments.value();
                Point2D point = edgeGeometry->getPoint(t);

                size_t pointIndex = result.points.size();
                result.points.push_back(point);
                result.tParameters.push_back({t});
                result.geometryIds.push_back({edgeId});
                edgePointIndices.push_back(pointIndex);
            }
        }

        // Add the end corner
        edgePointIndices.push_back(endCornerIdx);

        // Store the edge point indices
        result.edgeIdToPointIndicesMap[edgeId] = edgePointIndices;
    }

    return result;
}

} // namespace Meshing
