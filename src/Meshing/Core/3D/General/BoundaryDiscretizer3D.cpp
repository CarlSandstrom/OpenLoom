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

namespace Meshing
{

namespace
{
double angleBetweenTangents(const Vector3D& a, const Vector3D& b);
} // namespace

std::unique_ptr<DiscretizationResult3D>
BoundaryDiscretizer3D::discretize(const Geometry3D::GeometryCollection3D& geometry,
                                  const Topology3D::Topology3D& topology,
                                  const Geometry3D::DiscretizationSettings3D& settings,
                                  const std::optional<SizingField3D>& sizingField)
{
    auto result = std::make_unique<DiscretizationResult3D>();

    for (auto& cornerId : topology.getAllCornerIds())
    {
        auto corner = geometry.getCorner(cornerId);
        if (!corner) continue;

        size_t pointIndex = result->points.size();
        result->points.push_back(corner->getPoint());
        result->edgeParameters.push_back({});
        result->geometryIds.push_back({});
        result->cornerIdToPointIndexMap[cornerId] = pointIndex;
    }

    auto maxAngle = settings.getMaxAngleBetweenSegments();
    auto numberOfSegments = settings.getNumberOfSegmentsPerEdge();

    for (auto& edgeId : topology.getAllEdgeIds())
    {
        // Seam twins have no 3D geometry; they are filled in from their original edge below.
        if (topology.getSeamCollection().isSeamTwin(edgeId)) continue;

        auto& topologyEdge = topology.getEdge(edgeId);
        auto edge = geometry.getEdge(edgeId);
        if (!edge) continue;

        auto [tMin, tMax] = edge->getParameterBounds();

        size_t startIndex = result->cornerIdToPointIndexMap.at(topologyEdge.getStartCornerId());
        size_t endIndex = result->cornerIdToPointIndexMap.at(topologyEdge.getEndCornerId());

        result->edgeParameters[startIndex].push_back(tMin);
        result->geometryIds[startIndex].push_back(edgeId);
        result->edgeParameters[endIndex].push_back(tMax);
        result->geometryIds[endIndex].push_back(edgeId);

        std::vector<size_t> edgePointIndices;
        edgePointIndices.push_back(startIndex);

        if (edge->isDegenerate())
        {
            // A degenerate edge (e.g. a sphere's polar edge) has no real 3D curve, and walking
            // it inserts spurious coincident points. The pole is already its corner.
        }
        // Angle mode takes priority over fixed-count mode. A sizing field only ever adds
        // points: in fixed-count mode the count's uniform segment length is a maximum that
        // h(x) may shorten but never lengthen.
        else if (maxAngle.has_value() || sizingField.has_value())
        {
            std::optional<double> uniformSegmentLength;
            if (!maxAngle.has_value() && numberOfSegments.has_value() && numberOfSegments.value() > 0)
            {
                // Total arc length is not available analytically, so accumulate it with chord steps.
                constexpr size_t NUMBER_OF_LENGTH_STEPS = 1000;
                double totalLength = 0.0;
                Point3D previous = edge->getPoint(tMin);
                for (size_t i = 1; i <= NUMBER_OF_LENGTH_STEPS; ++i)
                {
                    double t = tMin + static_cast<double>(i) * (tMax - tMin) /
                                          static_cast<double>(NUMBER_OF_LENGTH_STEPS);
                    Point3D current = edge->getPoint(t);
                    totalLength += (current - previous).norm();
                    previous = current;
                }
                uniformSegmentLength = totalLength / static_cast<double>(numberOfSegments.value());
            }

            // Walk the edge in uniform parameter steps and emit a point once the accumulated
            // tangent angle reaches maxAngle or the accumulated arc length reaches the allowed
            // length. The length criterion is what bounds segments on a steep but barely
            // turning curve, and it also subdivides straight edges.
            constexpr size_t NUMBER_OF_STEPS = 1000;
            auto previousTangent = edge->getTangent(tMin);
            double accumulatedAngle = 0.0;

            Point3D previousPoint = edge->getPoint(tMin);
            Point3D segmentStartPoint = previousPoint;
            double accumulatedLength = 0.0;

            for (size_t i = 1; i <= NUMBER_OF_STEPS; ++i)
            {
                double t = tMin + static_cast<double>(i) * (tMax - tMin) / static_cast<double>(NUMBER_OF_STEPS);
                auto nextTangent = edge->getTangent(t);
                accumulatedAngle += angleBetweenTangents(previousTangent, nextTangent);
                previousTangent = nextTangent;

                Point3D point = edge->getPoint(t);
                accumulatedLength += (point - previousPoint).norm();
                previousPoint = point;

                bool angleReached = maxAngle.has_value() && accumulatedAngle >= maxAngle.value();

                bool lengthReached = false;
                if (sizingField.has_value() || uniformSegmentLength.has_value())
                {
                    double allowedLength = uniformSegmentLength.value_or(std::numeric_limits<double>::max());
                    if (sizingField.has_value())
                    {
                        // The smaller of both ends' sizes, so a segment leaving a coarse region
                        // into a fine one is cut to suit the fine end.
                        allowedLength = std::min({allowedLength,
                                                  sizingField->evaluate(segmentStartPoint),
                                                  sizingField->evaluate(point)});
                    }
                    lengthReached = accumulatedLength >= allowedLength;
                }

                // The end vertex is already a point of this edge; emitting there would
                // duplicate it (OPE-207).
                bool atEndVertex = t >= tMax;

                if ((angleReached || lengthReached) && !atEndVertex)
                {
                    size_t pointIndex = result->points.size();
                    result->points.push_back(point);
                    result->edgeParameters.push_back({t});
                    result->geometryIds.push_back({edgeId});
                    edgePointIndices.push_back(pointIndex);
                    accumulatedAngle = 0.0;
                    accumulatedLength = 0.0;
                    segmentStartPoint = point;
                }

                if (atEndVertex) break;
            }
        }
        else if (numberOfSegments.has_value() && numberOfSegments.value() > 1)
        {
            size_t n = numberOfSegments.value();
            for (size_t i = 1; i < n; ++i)
            {
                double t = tMin + (tMax - tMin) * static_cast<double>(i) / static_cast<double>(n);
                Point3D point = edge->getPoint(t);

                size_t pointIndex = result->points.size();
                result->points.push_back(point);
                result->edgeParameters.push_back({t});
                result->geometryIds.push_back({edgeId});
                edgePointIndices.push_back(pointIndex);
            }
        }

        edgePointIndices.push_back(endIndex);
        result->edgeIdToPointIndicesMap[edgeId] = edgePointIndices;
    }

    // A seam twin is the original seam edge reversed: the same curve at U + uPeriod in UV space.
    auto& seams = topology.getSeamCollection();
    for (auto& twinId : seams.getSeamTwinEdgeIds())
    {
        auto& originalId = seams.getOriginalEdgeId(twinId);
        auto originalIterator = result->edgeIdToPointIndicesMap.find(originalId);
        if (originalIterator == result->edgeIdToPointIndicesMap.end()) continue;

        auto reversed = originalIterator->second;
        std::reverse(reversed.begin(), reversed.end());
        result->edgeIdToPointIndicesMap[twinId] = std::move(reversed);
    }

    size_t surfaceSamples = settings.getNumberOfSamplesPerSurfaceDirection();

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

                size_t pointIndex = result->points.size();
                result->points.push_back(point);
                result->edgeParameters.push_back({});
                result->geometryIds.push_back({surfaceId});
                surfacePointIndices.push_back(pointIndex);
            }
        }

        result->surfaceIdToPointIndicesMap[surfaceId] = surfacePointIndices;
    }

    return result;
}

namespace
{

double angleBetweenTangents(const Vector3D& a, const Vector3D& b)
{
    double dot = std::clamp(a.dot(b), -1.0, 1.0);
    return std::acos(dot);
}

} // namespace

} // namespace Meshing
