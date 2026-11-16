#include "Meshing/Core/2D/BoundaryDiscretizer2D.h"
#include "Common/MathConstants.h"
#include "Geometry/2D/Base/GeometryCollection2D.h"
#include "Geometry/2D/Base/ICorner2D.h"
#include "Geometry/2D/Base/IEdge2D.h"
#include "Meshing/Core/2D/MeshingContext2D.h"
#include "Topology2D/Topology2D.h"
#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace Meshing
{

namespace
{

/// The two settings that decide how an edge is subdivided.
struct EdgeSpacing
{
    std::optional<double> maxAngle;
    std::optional<size_t> numberOfSegments;
};

size_t appendPoint(DiscretizationResult2D& result,
                   const Point2D& point,
                   std::vector<double> tParameters,
                   std::vector<std::string> geometryIds);
void addCorners(DiscretizationResult2D& result, const Geometry2D::GeometryCollection2D& geometry);
void addEdges(DiscretizationResult2D& result,
              const Geometry2D::GeometryCollection2D& geometry,
              const Topology2D::Topology2D& topology,
              const EdgeSpacing& spacing);
std::vector<size_t> addEdgeInteriorPoints(DiscretizationResult2D& result,
                                          const std::string& edgeId,
                                          const Geometry2D::IEdge2D& edge,
                                          const Point2D& startCornerPoint,
                                          const Point2D& endCornerPoint,
                                          const EdgeSpacing& spacing);
std::vector<size_t> addPointsByAngle(DiscretizationResult2D& result,
                                     const std::string& edgeId,
                                     const Geometry2D::IEdge2D& edge,
                                     const Point2D& startCornerPoint,
                                     const Point2D& endCornerPoint,
                                     double maxAngle);
std::vector<size_t> addPointsByCount(DiscretizationResult2D& result,
                                     const std::string& edgeId,
                                     const Geometry2D::IEdge2D& edge,
                                     size_t numberOfSegments);
double angleBetweenTangents(const Vector2D& first, const Vector2D& second);
Vector2D computeEdgeTangentAtParameter(const Geometry2D::IEdge2D& edge, double parameter);
} // namespace

BoundaryDiscretizer2D::BoundaryDiscretizer2D(const MeshingContext2D& context,
                                             const Geometry2D::DiscretizationSettings2D& settings) :
    context_(&context),
    settings_(settings)
{
}

DiscretizationResult2D BoundaryDiscretizer2D::discretize() const
{
    DiscretizationResult2D result;
    auto& geometry = context_->getGeometry();

    addCorners(result, geometry);
    addEdges(result,
             geometry,
             context_->getTopology(),
             {settings_.getMaxAngleBetweenSegments(), settings_.getNumberOfSegmentsPerEdge()});

    return result;
}

namespace
{

constexpr size_t NUMBER_OF_STEPS = 1000;

double parameterAtStep(double tMin, double tMax, size_t step)
{
    return tMin + static_cast<double>(step) * (tMax - tMin) / static_cast<double>(NUMBER_OF_STEPS);
}

size_t appendPoint(DiscretizationResult2D& result,
                   const Point2D& point,
                   std::vector<double> tParameters,
                   std::vector<std::string> geometryIds)
{
    size_t pointIndex = result.points.size();
    result.points.push_back(point);
    result.tParameters.push_back(std::move(tParameters));
    result.geometryIds.push_back(std::move(geometryIds));
    return pointIndex;
}

void addCorners(DiscretizationResult2D& result, const Geometry2D::GeometryCollection2D& geometry)
{
    for (auto& cornerId : geometry.getAllCornerIds())
    {
        auto corner = geometry.getCorner(cornerId);
        result.cornerIdToPointIndexMap[cornerId] = appendPoint(result, corner->getPoint(), {}, {});
    }
}

void addEdges(DiscretizationResult2D& result,
              const Geometry2D::GeometryCollection2D& geometry,
              const Topology2D::Topology2D& topology,
              const EdgeSpacing& spacing)
{
    for (auto& edgeId : topology.getAllEdgeIds())
    {
        auto& edgeTopology = topology.getEdge(edgeId);
        auto edgeGeometry = geometry.getEdge(edgeId);
        auto [tMin, tMax] = edgeGeometry->getParameterBounds();

        size_t startCornerIndex = result.cornerIdToPointIndexMap.at(edgeTopology.getStartCornerId());
        size_t endCornerIndex = result.cornerIdToPointIndexMap.at(edgeTopology.getEndCornerId());

        result.tParameters[startCornerIndex].push_back(tMin);
        result.geometryIds[startCornerIndex].push_back(edgeId);
        result.tParameters[endCornerIndex].push_back(tMax);
        result.geometryIds[endCornerIndex].push_back(edgeId);

        // Copies: appending interior points may reallocate result.points.
        Point2D startCornerPoint = result.points[startCornerIndex];
        Point2D endCornerPoint = result.points[endCornerIndex];

        std::vector<size_t> edgePointIndices{startCornerIndex};
        auto interiorIndices =
            addEdgeInteriorPoints(result, edgeId, *edgeGeometry, startCornerPoint, endCornerPoint, spacing);
        edgePointIndices.insert(edgePointIndices.end(), interiorIndices.begin(), interiorIndices.end());
        edgePointIndices.push_back(endCornerIndex);
        result.edgeIdToPointIndicesMap[edgeId] = std::move(edgePointIndices);
    }
}

// Angle mode takes priority over fixed-count mode.
std::vector<size_t> addEdgeInteriorPoints(DiscretizationResult2D& result,
                                          const std::string& edgeId,
                                          const Geometry2D::IEdge2D& edge,
                                          const Point2D& startCornerPoint,
                                          const Point2D& endCornerPoint,
                                          const EdgeSpacing& spacing)
{
    if (spacing.maxAngle.has_value())
        return addPointsByAngle(result, edgeId, edge, startCornerPoint, endCornerPoint, spacing.maxAngle.value());

    if (spacing.numberOfSegments.has_value() && spacing.numberOfSegments.value() > 1)
        return addPointsByCount(result, edgeId, edge, spacing.numberOfSegments.value());

    return {};
}

// Walk the edge in uniform parameter steps and emit a point once the tangent has turned
// by maxAngle since the last emitted point.
std::vector<size_t> addPointsByAngle(DiscretizationResult2D& result,
                                     const std::string& edgeId,
                                     const Geometry2D::IEdge2D& edge,
                                     const Point2D& startCornerPoint,
                                     const Point2D& endCornerPoint,
                                     double maxAngle)
{
    auto [tMin, tMax] = edge.getParameterBounds();
    auto previousTangent = computeEdgeTangentAtParameter(edge, tMin);

    std::vector<size_t> pointIndices;
    for (size_t step = 1; step <= NUMBER_OF_STEPS; ++step)
    {
        double t = parameterAtStep(tMin, tMax, step);
        auto nextTangent = computeEdgeTangentAtParameter(edge, t);

        if (angleBetweenTangents(previousTangent, nextTangent) >= maxAngle)
        {
            Point2D point = edge.getPoint(t);

            previousTangent = nextTangent;

            // A point on either corner would duplicate it.
            if ((point - startCornerPoint).norm() < 1e-6 || (point - endCornerPoint).norm() < 1e-6) continue;

            pointIndices.push_back(appendPoint(result, point, {t}, {edgeId}));
        }

        if (t >= tMax - EPSILON) break;
    }
    return pointIndices;
}

std::vector<size_t> addPointsByCount(DiscretizationResult2D& result,
                                     const std::string& edgeId,
                                     const Geometry2D::IEdge2D& edge,
                                     size_t numberOfSegments)
{
    auto [tMin, tMax] = edge.getParameterBounds();

    std::vector<size_t> pointIndices;
    for (size_t i = 1; i < numberOfSegments; ++i)
    {
        double t = tMin + static_cast<double>(i) * (tMax - tMin) / static_cast<double>(numberOfSegments);
        pointIndices.push_back(appendPoint(result, edge.getPoint(t), {t}, {edgeId}));
    }
    return pointIndices;
}

// The unsigned difference of the tangents' polar angles, folded into [0, pi].
double angleBetweenTangents(const Vector2D& first, const Vector2D& second)
{
    double angle = std::fabs(std::atan2(second.y(), second.x()) - std::atan2(first.y(), first.x()));
    if (angle > PI) angle = 2 * PI - angle;
    return angle;
}

// A central difference over EPSILON, clamped to the edge's parameter range.
Vector2D computeEdgeTangentAtParameter(const Geometry2D::IEdge2D& edge, double parameter)
{
    auto bounds = edge.getParameterBounds();
    double parameterBefore = std::max(bounds.first, std::min(bounds.second, parameter - EPSILON));
    double parameterAfter = std::max(bounds.first, std::min(bounds.second, parameter + EPSILON));

    Point2D pointBefore = edge.getPoint(parameterBefore);
    Point2D pointAfter = edge.getPoint(parameterAfter);

    return (pointAfter - pointBefore).normalized();
}

} // namespace

} // namespace Meshing
