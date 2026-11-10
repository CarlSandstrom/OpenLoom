#include "Meshing/Core/3D/General/BoundaryDiscretizer3D.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ICorner3D.h"
#include "Geometry/3D/Base/IEdge3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Topology/Topology3D.h"
#include <algorithm>
#include <cmath>
#include <limits>
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

size_t appendPoint(DiscretizationResult3D& result,
                   const Point3D& point,
                   std::vector<double> edgeParameters,
                   std::vector<std::string> geometryIds);
void addCorners(DiscretizationResult3D& result,
                const Geometry3D::GeometryCollection3D& geometry,
                const Topology3D::Topology3D& topology);
void addEdges(DiscretizationResult3D& result,
              const Geometry3D::GeometryCollection3D& geometry,
              const Topology3D::Topology3D& topology,
              const EdgeSpacing& spacing,
              const std::optional<SizingField3D>& sizingField);
void addSeamTwins(DiscretizationResult3D& result, const Topology3D::Topology3D& topology);
void addSurfaceSamples(DiscretizationResult3D& result,
                       const Geometry3D::GeometryCollection3D& geometry,
                       const Topology3D::Topology3D& topology,
                       size_t surfaceSamples);
std::vector<size_t> addEdgeInteriorPoints(DiscretizationResult3D& result,
                                          const std::string& edgeId,
                                          const Geometry3D::IEdge3D& edge,
                                          const EdgeSpacing& spacing,
                                          const std::optional<SizingField3D>& sizingField);
std::vector<size_t> addPointsByWalk(DiscretizationResult3D& result,
                                    const std::string& edgeId,
                                    const Geometry3D::IEdge3D& edge,
                                    const EdgeSpacing& spacing,
                                    const std::optional<SizingField3D>& sizingField);
std::vector<size_t> addPointsByCount(DiscretizationResult3D& result,
                                     const std::string& edgeId,
                                     const Geometry3D::IEdge3D& edge,
                                     size_t numberOfSegments);
std::optional<double> measureUniformSegmentLength(const Geometry3D::IEdge3D& edge, const EdgeSpacing& spacing);
std::optional<double> findAllowedLength(const std::optional<double>& uniformSegmentLength,
                                        const std::optional<SizingField3D>& sizingField,
                                        const Point3D& segmentStartPoint,
                                        const Point3D& point);
double angleBetweenTangents(const Vector3D& first, const Vector3D& second);
} // namespace

std::unique_ptr<DiscretizationResult3D>
BoundaryDiscretizer3D::discretize(const Geometry3D::GeometryCollection3D& geometry,
                                  const Topology3D::Topology3D& topology,
                                  const Geometry3D::DiscretizationSettings3D& settings,
                                  const std::optional<SizingField3D>& sizingField)
{
    auto result = std::make_unique<DiscretizationResult3D>();

    addCorners(*result, geometry, topology);
    addEdges(*result,
             geometry,
             topology,
             {settings.getMaxAngleBetweenSegments(), settings.getNumberOfSegmentsPerEdge()},
             sizingField);
    addSeamTwins(*result, topology);
    addSurfaceSamples(*result, geometry, topology, settings.getNumberOfSamplesPerSurfaceDirection());

    return result;
}

namespace
{

constexpr size_t NUMBER_OF_STEPS = 1000;

double parameterAtStep(double tMin, double tMax, size_t step)
{
    return tMin + static_cast<double>(step) * (tMax - tMin) / static_cast<double>(NUMBER_OF_STEPS);
}

size_t appendPoint(DiscretizationResult3D& result,
                   const Point3D& point,
                   std::vector<double> edgeParameters,
                   std::vector<std::string> geometryIds)
{
    size_t pointIndex = result.points.size();
    result.points.push_back(point);
    result.edgeParameters.push_back(std::move(edgeParameters));
    result.geometryIds.push_back(std::move(geometryIds));
    return pointIndex;
}

void addCorners(DiscretizationResult3D& result,
                const Geometry3D::GeometryCollection3D& geometry,
                const Topology3D::Topology3D& topology)
{
    for (auto& cornerId : topology.getAllCornerIds())
    {
        auto corner = geometry.getCorner(cornerId);
        if (!corner) continue;

        result.cornerIdToPointIndexMap[cornerId] = appendPoint(result, corner->getPoint(), {}, {});
    }
}

void addEdges(DiscretizationResult3D& result,
              const Geometry3D::GeometryCollection3D& geometry,
              const Topology3D::Topology3D& topology,
              const EdgeSpacing& spacing,
              const std::optional<SizingField3D>& sizingField)
{
    for (auto& edgeId : topology.getAllEdgeIds())
    {
        // Seam twins have no 3D geometry; addSeamTwins fills them in from their original edge.
        if (topology.getSeamCollection().isSeamTwin(edgeId)) continue;

        auto& topologyEdge = topology.getEdge(edgeId);
        auto edge = geometry.getEdge(edgeId);
        if (!edge) continue;

        auto [tMin, tMax] = edge->getParameterBounds();

        size_t startIndex = result.cornerIdToPointIndexMap.at(topologyEdge.getStartCornerId());
        size_t endIndex = result.cornerIdToPointIndexMap.at(topologyEdge.getEndCornerId());

        result.edgeParameters[startIndex].push_back(tMin);
        result.geometryIds[startIndex].push_back(edgeId);
        result.edgeParameters[endIndex].push_back(tMax);
        result.geometryIds[endIndex].push_back(edgeId);

        std::vector<size_t> edgePointIndices;
        edgePointIndices.push_back(startIndex);
        auto interiorIndices = addEdgeInteriorPoints(result, edgeId, *edge, spacing, sizingField);
        edgePointIndices.insert(edgePointIndices.end(), interiorIndices.begin(), interiorIndices.end());
        edgePointIndices.push_back(endIndex);
        result.edgeIdToPointIndicesMap[edgeId] = std::move(edgePointIndices);
    }
}

// A seam twin is the original seam edge reversed: the same curve at U + uPeriod in UV space.
void addSeamTwins(DiscretizationResult3D& result, const Topology3D::Topology3D& topology)
{
    auto& seams = topology.getSeamCollection();
    for (auto& twinId : seams.getSeamTwinEdgeIds())
    {
        auto& originalId = seams.getOriginalEdgeId(twinId);
        auto originalIterator = result.edgeIdToPointIndicesMap.find(originalId);
        if (originalIterator == result.edgeIdToPointIndicesMap.end()) continue;

        auto reversed = originalIterator->second;
        std::reverse(reversed.begin(), reversed.end());
        result.edgeIdToPointIndicesMap[twinId] = std::move(reversed);
    }
}

void addSurfaceSamples(DiscretizationResult3D& result,
                       const Geometry3D::GeometryCollection3D& geometry,
                       const Topology3D::Topology3D& topology,
                       size_t surfaceSamples)
{
    for (auto& surfaceId : topology.getAllSurfaceIds())
    {
        auto surface = geometry.getSurface(surfaceId);
        if (!surface) continue;

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

                // The untrimmed parameter grid can land inside a hole in the face (OPE-169).
                if (!surface->isPointWithinTrimmedBoundary(point)) continue;

                surfacePointIndices.push_back(appendPoint(result, point, {}, {surfaceId}));
            }
        }

        result.surfaceIdToPointIndicesMap[surfaceId] = surfacePointIndices;
    }
}

std::vector<size_t> addEdgeInteriorPoints(DiscretizationResult3D& result,
                                          const std::string& edgeId,
                                          const Geometry3D::IEdge3D& edge,
                                          const EdgeSpacing& spacing,
                                          const std::optional<SizingField3D>& sizingField)
{
    // A degenerate edge (e.g. a sphere's polar edge) has no real 3D curve, and walking
    // it inserts spurious coincident points. The pole is already its corner.
    if (edge.isDegenerate()) return {};

    // Angle mode takes priority over fixed-count mode. A sizing field only ever adds
    // points: in fixed-count mode the count's uniform segment length is a maximum that
    // h(x) may shorten but never lengthen.
    if (spacing.maxAngle.has_value() || sizingField.has_value())
        return addPointsByWalk(result, edgeId, edge, spacing, sizingField);

    if (spacing.numberOfSegments.has_value() && spacing.numberOfSegments.value() > 1)
        return addPointsByCount(result, edgeId, edge, spacing.numberOfSegments.value());

    return {};
}

// Walk the edge in uniform parameter steps and emit a point once the accumulated
// tangent angle reaches maxAngle or the accumulated arc length reaches the allowed
// length. The length criterion is what bounds segments on a steep but barely
// turning curve, and it also subdivides straight edges.
std::vector<size_t> addPointsByWalk(DiscretizationResult3D& result,
                                    const std::string& edgeId,
                                    const Geometry3D::IEdge3D& edge,
                                    const EdgeSpacing& spacing,
                                    const std::optional<SizingField3D>& sizingField)
{
    auto [tMin, tMax] = edge.getParameterBounds();
    auto uniformSegmentLength = measureUniformSegmentLength(edge, spacing);

    std::vector<size_t> pointIndices;
    auto previousTangent = edge.getTangent(tMin);
    double accumulatedAngle = 0.0;

    Point3D previousPoint = edge.getPoint(tMin);
    Point3D segmentStartPoint = previousPoint;
    double accumulatedLength = 0.0;

    for (size_t step = 1; step <= NUMBER_OF_STEPS; ++step)
    {
        double t = parameterAtStep(tMin, tMax, step);
        auto nextTangent = edge.getTangent(t);
        accumulatedAngle += angleBetweenTangents(previousTangent, nextTangent);
        previousTangent = nextTangent;

        Point3D point = edge.getPoint(t);
        accumulatedLength += (point - previousPoint).norm();
        previousPoint = point;

        bool angleReached = spacing.maxAngle.has_value() && accumulatedAngle >= spacing.maxAngle.value();

        auto allowedLength = findAllowedLength(uniformSegmentLength, sizingField, segmentStartPoint, point);
        bool lengthReached = allowedLength.has_value() && accumulatedLength >= allowedLength.value();

        // The end vertex is already a point of this edge; emitting there would
        // duplicate it (OPE-207).
        bool atEndVertex = t >= tMax;

        if ((angleReached || lengthReached) && !atEndVertex)
        {
            pointIndices.push_back(appendPoint(result, point, {t}, {edgeId}));
            accumulatedAngle = 0.0;
            accumulatedLength = 0.0;
            segmentStartPoint = point;
        }

        if (atEndVertex) break;
    }
    return pointIndices;
}

std::vector<size_t> addPointsByCount(DiscretizationResult3D& result,
                                     const std::string& edgeId,
                                     const Geometry3D::IEdge3D& edge,
                                     size_t numberOfSegments)
{
    auto [tMin, tMax] = edge.getParameterBounds();

    std::vector<size_t> pointIndices;
    for (size_t i = 1; i < numberOfSegments; ++i)
    {
        double t = tMin + (tMax - tMin) * static_cast<double>(i) / static_cast<double>(numberOfSegments);
        pointIndices.push_back(appendPoint(result, edge.getPoint(t), {t}, {edgeId}));
    }
    return pointIndices;
}

// Fixed-count mode under a sizing field: the count's uniform segment length. Total arc
// length is not available analytically, so it is accumulated with chord steps.
std::optional<double> measureUniformSegmentLength(const Geometry3D::IEdge3D& edge, const EdgeSpacing& spacing)
{
    if (spacing.maxAngle.has_value()) return std::nullopt;
    if (!spacing.numberOfSegments.has_value() || spacing.numberOfSegments.value() == 0) return std::nullopt;

    auto [tMin, tMax] = edge.getParameterBounds();
    double totalLength = 0.0;
    Point3D previous = edge.getPoint(tMin);
    for (size_t step = 1; step <= NUMBER_OF_STEPS; ++step)
    {
        Point3D current = edge.getPoint(parameterAtStep(tMin, tMax, step));
        totalLength += (current - previous).norm();
        previous = current;
    }
    return totalLength / static_cast<double>(spacing.numberOfSegments.value());
}

// Empty when neither a uniform length nor a sizing field bounds the segment.
std::optional<double> findAllowedLength(const std::optional<double>& uniformSegmentLength,
                                        const std::optional<SizingField3D>& sizingField,
                                        const Point3D& segmentStartPoint,
                                        const Point3D& point)
{
    if (!sizingField.has_value() && !uniformSegmentLength.has_value()) return std::nullopt;

    double allowedLength = uniformSegmentLength.value_or(std::numeric_limits<double>::max());
    if (!sizingField.has_value()) return allowedLength;

    // The smaller of both ends' sizes, so a segment leaving a coarse region
    // into a fine one is cut to suit the fine end.
    return std::min({allowedLength, sizingField->evaluate(segmentStartPoint), sizingField->evaluate(point)});
}

double angleBetweenTangents(const Vector3D& first, const Vector3D& second)
{
    double dot = std::clamp(first.dot(second), -1.0, 1.0);
    return std::acos(dot);
}

} // namespace

} // namespace Meshing
