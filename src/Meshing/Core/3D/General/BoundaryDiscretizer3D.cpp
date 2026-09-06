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

double angleBetweenTangents(const std::array<double, 3>& a, const std::array<double, 3>& b)
{
    double dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    dot = std::clamp(dot, -1.0, 1.0);
    return std::acos(dot);
}

} // namespace

BoundaryDiscretizer3D::BoundaryDiscretizer3D(const Geometry3D::GeometryCollection3D& geometry,
                                             const Topology3D::Topology3D& topology,
                                             const Geometry3D::DiscretizationSettings3D& settings,
                                             const SizingField3D* sizingField) :
    geometry_(&geometry),
    topology_(&topology),
    settings_(settings),
    sizingField_(sizingField),
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

    // The sizing field supplies the length bound the angle criterion cannot
    // express (see the class comment). Supplied by the caller so that every
    // consumer of h(x) reads one field.
    const SizingField3D* const sizingField = sizingField_;

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

        // A degenerate edge (e.g. a sphere's polar edge) has no real 3D curve —
        // getPoint()/getTangent() are not meaningful along it, and walking it
        // as if it were a normal curve inserts hundreds of spurious coincident
        // points at the singularity. It contributes no interior points; the
        // pole is already represented by its corner.
        if (edge->isDegenerate())
        {
            // no interior points
        }
        // Angle mode takes priority over fixed-count mode (see
        // DiscretizationSettings3D), and a sizing field refines whichever
        // mode is active rather than replacing it: in angle mode it adds the
        // length criterion to the walk, and in fixed-count mode it treats
        // the count's uniform segment length as a maximum that h(x) may
        // shorten but never lengthen. The field only ever ADDS points --
        // a count is a coarseness request, not a protection guarantee, and
        // CurveProtectionSubdivider already densifies past it wherever the
        // protection properties demand (so the count was never authoritative
        // to begin with).
        else if (maxAngle.has_value() || sizingField != nullptr)
        {
            // In fixed-count mode the count contributes its uniform segment
            // arc length as a cap on the walk's length criterion. Total arc
            // length is not available analytically, so accumulate it with the
            // same chord stepping the walk itself uses.
            std::optional<double> uniformSegmentLength;
            if (!maxAngle.has_value() && numSegments.has_value() && numSegments.value() > 0)
            {
                constexpr size_t NUM_LENGTH_STEPS = 1000;
                double totalLength = 0.0;
                Point3D previous = edge->getPoint(tMin);
                for (size_t i = 1; i <= NUM_LENGTH_STEPS; ++i)
                {
                    const double t = tMin + static_cast<double>(i) * (tMax - tMin) /
                                                static_cast<double>(NUM_LENGTH_STEPS);
                    const Point3D current = edge->getPoint(t);
                    totalLength += (current - previous).norm();
                    previous = current;
                }
                uniformSegmentLength = totalLength / static_cast<double>(numSegments.value());
            }
            // Angle-based: walk 1000 uniform steps; insert a point whenever the
            // accumulated tangent-angle change since the last inserted point
            // meets or exceeds maxAngle. Straight edges produce no interior points.
            //
            // With a sizing field, accumulated ARCLENGTH is tracked alongside
            // the angle and either criterion alone triggers a point. That is
            // what bounds segment length on a curve that is steep but barely
            // turning, where the angle criterion permits a segment of any
            // length at all (see the class comment). A straight edge still
            // produces no interior points from the angle term, but the length
            // term does subdivide it -- which is the point: a long straight
            // crease needs interior points for its protecting balls to stay
            // proportionate to the local mesh size.
            constexpr size_t NUM_STEPS = 1000;
            auto prevTangent = edge->getTangent(tMin);
            double accumulated = 0.0;

            Point3D prevPoint = edge->getPoint(tMin);
            Point3D segmentStartPoint = prevPoint;
            double accumulatedLength = 0.0;

            for (size_t i = 1; i <= NUM_STEPS; ++i)
            {
                double t = tMin + static_cast<double>(i) * (tMax - tMin) / static_cast<double>(NUM_STEPS);
                auto nextTangent = edge->getTangent(t);
                accumulated += angleBetweenTangents(prevTangent, nextTangent);
                prevTangent = nextTangent;

                const Point3D point = edge->getPoint(t);
                accumulatedLength += (point - prevPoint).norm();
                prevPoint = point;

                const bool angleReached = maxAngle.has_value() && accumulated >= maxAngle.value();

                // Bound the segment by the SMALLER of the sizes its two ends
                // ask for, so a segment running from a coarse region into a
                // fine one is cut to suit the fine end rather than overshoot
                // it.
                bool lengthReached = false;
                if (sizingField != nullptr || uniformSegmentLength.has_value())
                {
                    double allowedLength = uniformSegmentLength.value_or(std::numeric_limits<double>::max());
                    if (sizingField != nullptr)
                        allowedLength = std::min({allowedLength,
                                                  sizingField->evaluate(segmentStartPoint),
                                                  sizingField->evaluate(point)});
                    lengthReached = accumulatedLength >= allowedLength;
                }

                if (angleReached || lengthReached)
                {
                    size_t pointIndex = result_->points.size();
                    result_->points.push_back(point);
                    result_->edgeParameters.push_back({t});
                    result_->geometryIds.push_back({edgeId});
                    edgePointIndices.push_back(pointIndex);
                    accumulated = 0.0;
                    accumulatedLength = 0.0;
                    segmentStartPoint = point;
                }

                if (t >= tMax)
                    break;
            }
        }
        else if (numSegments.has_value() && numSegments.value() > 1)
        {
            // Fixed-count without a sizing field: divide the edge into
            // numSegments uniform segments in parameter space.
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

                // A uniform grid over the surface's untrimmed parameter
                // rectangle can land inside a hole or other cutout in the
                // face -- skip any sample that isn't actually on the
                // trimmed patch (see OPE-169).
                if (!surface->isPointWithinTrimmedBoundary(point))
                    continue;

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
