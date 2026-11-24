#include "Meshing/Core/3D/RCDT/ProtectingBallPlacer.h"

#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/IEdge3D.h"
#include "Meshing/Core/3D/General/NonIncidentSurfaceDistance.h"
#include "Meshing/Data/3D/DiscretizationResult3D.h"
#include "Topology/SeamCollection.h"
#include "Topology/Topology3D.h"
#include "spdlog/spdlog.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace Meshing
{

namespace
{

// CGAL Mesh_3's constants (Protect_edges_sizing_field.h).
constexpr double DISTANCE_DIVISOR = 2.1;
constexpr int MAXIMUM_REPAIR_ROUNDS = 29;
constexpr std::size_t REEVALUATE_SIZE_ABOVE = 10;

// Beyond CGAL: neighbours along a curve sit this many radii apart, where CGAL
// spaces them one radius apart. Below 2 they still overlap, and above 1 neither
// centre lies inside the other's ball.
constexpr double NEIGHBOUR_SPACING_FACTOR = 1.8;

// A ball reaches at most half way to the nearest surface sharing no corner
// with it, so the facets of that surface stay clear of the ball.
constexpr double SURFACE_DISTANCE_DIVISOR = 2.0;

// Floor on that cap, as a fraction of the model's bounding-box diagonal: a
// curve lying on a surface it shares no corner with (an invalid B-rep) would
// otherwise ask for balls of no size, and repair would never settle. It is
// also CGAL's minimal_size: no ball is placed smaller, and repair's shrink
// stops there, for two curves touching away from a corner (OPE-249).
constexpr double MINIMUM_CAP_FRACTION = 1e-3;

// A corner nearer a curve's end than this fraction of the curve's length is on
// it: evaluating a closed curve at its end lands ~1e-15 off its corner, and
// counting that as an offset flips the rounded ball count on symmetric models.
constexpr double OFF_CURVE_CORNER_FRACTION = 1e-9;

// Relative slack within which a ball's radius counts as sitting at its cap.
constexpr double AT_CAP_TOLERANCE = 1e-9;

/// One protecting ball on a curve, by arc length from the curve's start.
struct Ball
{
    double arc;
    Point3D point;
    double radius;
    std::string cornerId; // empty for a ball inside the curve
    bool atMinimumSize = false;
};

/// The cap on a ball's radius from the surfaces sharing no corner with it.
struct SurfaceCap
{
    const NonIncidentSurfaceDistance* surfaceDistance;
    double floor;
};

/// The largest radius a ball at point may have, for a ball belonging to the
/// named corners: its own, or the two ends of its curve.
double capAt(const SurfaceCap& cap, const Point3D& point, const std::vector<std::string>& cornerIds)
{
    auto radius = cap.surfaceDistance->distance(point, cornerIds) / SURFACE_DISTANCE_DIVISOR;
    if (radius >= cap.floor) return radius;
    spdlog::warn("ProtectingBallPlacer: a surface sharing no corner with the ball at ({}, {}, {}) is {} away; "
                 "its radius is floored at {}",
                 point.x(), point.y(), point.z(), radius * SURFACE_DISTANCE_DIVISOR, cap.floor);
    return cap.floor;
}

/// A protected curve: its geometry, the size function read from the existing
/// discretization's spacing, the cap on any of its balls' radii, and its balls
/// in order from start to end.
struct Curve
{
    std::string edgeId;
    const Geometry3D::IEdge3D* edge;
    SurfaceCap cap;
    std::vector<std::string> cornerIds;
    double length;
    std::vector<double> sizeBreaks; // arc positions of the discretization's points
    std::vector<Ball> balls;
};

double parameterAt(const Curve& curve, double arc)
{
    auto [start, end] = curve.edge->getParameterBounds();
    return curve.edge->getParameterAtArcLengthFraction(start, end, arc / curve.length);
}

Point3D pointAt(const Curve& curve, double arc)
{
    return curve.edge->getPoint(parameterAt(curve, arc));
}

/// The discretization's spacing at an arc position, capped by the surfaces near
/// it, and never below the minimum size.
double sizeAt(const Curve& curve, double arc)
{
    auto& breaks = curve.sizeBreaks;
    auto upper = std::upper_bound(breaks.begin(), breaks.end(), arc);
    auto segment = std::clamp<std::size_t>(upper - breaks.begin(), 1, breaks.size() - 1);
    auto size = std::min(breaks[segment] - breaks[segment - 1], capAt(curve.cap, pointAt(curve, arc), curve.cornerIds));
    return std::max(size, curve.cap.floor);
}

/// The existing discretization's points along the curve, as arc positions
/// scaled so the last one is the curve's length.
std::vector<double> sizeBreaksOf(const std::vector<std::size_t>& chain,
                                 const std::vector<Point3D>& points,
                                 double length)
{
    std::vector<double> breaks{0.0};
    for (std::size_t i = 1; i < chain.size(); ++i)
        breaks.push_back(breaks.back() + (points[chain[i]] - points[chain[i - 1]]).norm());
    auto chordLength = breaks.back();
    for (double& value : breaks)
        value *= length / chordLength;
    return breaks;
}

/// ballCount balls between startBall and endBall (a nonzero count is bumped for
/// a gap wider than two radii, or for a closed loop's single ball), radii
/// growing linearly from the smaller end radius to the larger and each ball
/// spaced from the last by its own radius.
std::vector<Ball> spacedBalls(const Curve& curve,
                              const Ball& startBall,
                              const Ball& endBall,
                              bool closedLoop,
                              std::size_t ballCount)
{
    auto span = endBall.arc - startBall.arc;
    auto forward = startBall.radius <= endBall.radius;
    auto smallRadius = std::min(startBall.radius, endBall.radius);
    auto largeRadius = std::max(startBall.radius, endBall.radius);

    auto radiusIncrement = (largeRadius - smallRadius) / static_cast<double>(ballCount + 1);
    auto covered = smallRadius * static_cast<double>(ballCount + 1) +
                   static_cast<double>((ballCount + 1) * (ballCount + 2)) / 2.0 * radiusIncrement;
    auto fraction = span / covered;
    double step = smallRadius + radiusIncrement;
    double normalizedStep = fraction * step;
    double distance = normalizedStep;
    if (ballCount == 0 && span >= smallRadius + largeRadius)
    {
        ballCount = 1;
        step = smallRadius + (span - smallRadius - largeRadius) / 2.0;
        distance = step;
        normalizedStep = step;
    }
    else if (closedLoop && ballCount == 1)
    {
        ballCount = 2;
        step = span / 3.0;
        distance = step;
        normalizedStep = step;
    }

    std::vector<Ball> balls;
    for (std::size_t i = 1; i <= ballCount; ++i)
    {
        auto arc = forward ? startBall.arc + distance : endBall.arc - distance;
        auto point = pointAt(curve, arc);
        auto radius = std::max(std::min({normalizedStep, smallRadius + distance / span * (largeRadius - smallRadius),
                                         capAt(curve.cap, point, curve.cornerIds)}),
                               curve.cap.floor);
        balls.push_back({arc, point, radius, {}});
        if (radiusIncrement > 0.0)
        {
            step += radiusIncrement;
            normalizedStep = fraction * step;
        }
        distance += normalizedStep;
    }
    if (!forward)
        std::reverse(balls.begin(), balls.end());
    return balls;
}

/// CGAL's insert_balls: the balls strictly between startBall and endBall
/// (startBall.arc < endBall.arc), in increasing arc order.
std::vector<Ball> insertBalls(const Curve& curve, const Ball& startBall, const Ball& endBall, bool closedLoop)
{
    auto span = endBall.arc - startBall.arc;
    if (span <= 0.0)
        return {};
    auto smallRadius = std::min(startBall.radius, endBall.radius);
    auto largeRadius = std::max(startBall.radius, endBall.radius);
    auto estimate = std::ceil(2.0 * (span - NEIGHBOUR_SPACING_FACTOR * largeRadius) /
                              (NEIGHBOUR_SPACING_FACTOR * (smallRadius + largeRadius)));
    std::size_t ballCount = estimate > 0.0 ? static_cast<std::size_t>(estimate) : 0;

    // Place a ball at the midpoint with the local size first, so the size
    // function is followed instead of interpolated between the ends: CGAL
    // does so for a long run; it must also happen wherever interpolation
    // would oversize the middle. CGAL's sizing fields vary slowly, but a
    // spacing read off an angle-based discretization does not -- on the dense
    // saddle's end parabolas it falls from 1.33 at the corners to 0.028 at
    // the apex, and interpolating between the corners put balls of radius
    // 0.8 there (OPE-186). The midpoint must lie outside both end balls, or
    // its ball would be hidden.
    auto middleArc = startBall.arc + span / 2.0;
    Ball middle{middleArc, pointAt(curve, middleArc), sizeAt(curve, middleArc), {}};
    auto oversizedMiddle = ballCount >= 1 && middle.radius < (smallRadius + largeRadius) / 2.0 &&
                           (middle.point - startBall.point).norm() > startBall.radius &&
                           (middle.point - endBall.point).norm() > endBall.radius;
    if (ballCount < REEVALUATE_SIZE_ABOVE && !oversizedMiddle)
        return spacedBalls(curve, startBall, endBall, closedLoop, ballCount);

    auto balls = insertBalls(curve, startBall, middle, false);
    balls.push_back(middle);
    auto second = insertBalls(curve, middle, endBall, false);
    balls.insert(balls.end(), second.begin(), second.end());
    return balls;
}

/// The ball at the end of a curve at arc (0 or its length): its corner's, with
/// the radius it reaches along the curve. Within a STEP file's tolerance a
/// corner may sit off the curve's end -- 0.174 on Flanged_Housing.step -- and
/// the ball then covers the curve only out to its radius less that offset.
/// Spacing against that radius keeps the corner overlapping its neighbour,
/// whose distance from the corner is at most its arc plus the offset (OPE-250).
Ball cornerBall(const Curve& curve, const std::string& cornerId, double arc, const std::map<std::string, Ball>& corners)
{
    Ball ball = corners.at(cornerId);
    ball.arc = arc;
    auto offset = (pointAt(curve, arc) - ball.point).norm();
    if (offset > OFF_CURVE_CORNER_FRACTION * curve.length)
        ball.radius = std::max(ball.radius - offset, 0.0);
    return ball;
}

/// The curve's balls end to end, corners included.
std::vector<Ball> withCorners(const Curve& curve,
                              const std::string& startCorner,
                              const std::string& endCorner,
                              const std::map<std::string, Ball>& corners)
{
    std::vector<Ball> all{cornerBall(curve, startCorner, 0.0, corners)};
    all.insert(all.end(), curve.balls.begin(), curve.balls.end());
    all.push_back(cornerBall(curve, endCorner, curve.length, corners));
    return all;
}

/// Adds balls wherever two neighbours along the curve no longer overlap
/// (arc length beyond the sum of their radii -- CGAL's sufficient condition).
/// Returns whether any were added.
bool repopulate(Curve& curve,
                const std::string& startCorner,
                const std::string& endCorner,
                const std::map<std::string, Ball>& corners)
{
    auto all = withCorners(curve, startCorner, endCorner, corners);
    auto closedLoop = startCorner == endCorner;
    std::vector<Ball> interior;
    bool added = false;
    for (std::size_t i = 0; i + 1 < all.size(); ++i)
    {
        if (i > 0)
            interior.push_back(all[i]);
        if (all[i + 1].arc - all[i].arc <= all[i].radius + all[i + 1].radius)
            continue;
        auto filler = insertBalls(curve, all[i], all[i + 1], closedLoop && all.size() == 2);
        interior.insert(interior.end(), filler.begin(), filler.end());
        added = added || !filler.empty();
    }
    curve.balls = std::move(interior);
    return added;
}

struct CurveEnds
{
    std::string startCorner;
    std::string endCorner;
};

/// A handle on one ball: a corner, or a ball inside a curve.
struct BallReference
{
    std::string cornerId;
    std::size_t curve = 0;
    std::size_t index = 0;
};

using BallEntry = std::pair<BallReference, Ball*>;

std::string ballKey(const BallReference& reference)
{
    return reference.cornerId.empty() ?
               "c" + std::to_string(reference.curve) + ":" + std::to_string(reference.index) :
               "v" + reference.cornerId;
}

/// Every ball, corners first, then each curve's in order.
std::vector<BallEntry> collectBalls(std::vector<Curve>& curves, std::map<std::string, Ball>& corners)
{
    std::vector<BallEntry> balls;
    for (auto& [cornerId, ball] : corners)
        balls.push_back({{cornerId, 0, 0}, &ball});
    for (std::size_t curveIndex = 0; curveIndex < curves.size(); ++curveIndex)
        for (std::size_t i = 0; i < curves[curveIndex].balls.size(); ++i)
            balls.push_back({{"", curveIndex, i}, &curves[curveIndex].balls[i]});
    return balls;
}

/// Neighbours: consecutive along a curve, corners included, as pairs of ball keys.
std::set<std::pair<std::string, std::string>> neighbourKeys(const std::vector<Curve>& curves,
                                                            const std::vector<CurveEnds>& ends)
{
    std::set<std::pair<std::string, std::string>> neighbours;
    for (std::size_t curveIndex = 0; curveIndex < curves.size(); ++curveIndex)
    {
        std::vector<std::string> chain{"v" + ends[curveIndex].startCorner};
        for (std::size_t i = 0; i < curves[curveIndex].balls.size(); ++i)
            chain.push_back(ballKey({"", curveIndex, i}));
        chain.push_back("v" + ends[curveIndex].endCorner);
        for (std::size_t i = 0; i + 1 < chain.size(); ++i)
            neighbours.insert(std::minmax(chain[i], chain[i + 1]));
    }
    return neighbours;
}

/// One shrink pass over every intersecting pair, all against the radii as they
/// stood at the start. Returns whether any radius changed.
///
/// CGAL's minimal_size: a ball a non-neighbour would shrink below minimumSize
/// gets exactly minimumSize instead -- raised, if it was smaller -- and is
/// never shrunk again. Without it, two curves that touch away from a corner
/// shrink each other's balls without bound and repair never settles
/// (OPE-249). The radius is exact, not merely floored, so that balls at it
/// all weigh the same and none hides another.
bool shrinkIntersectingBalls(const std::vector<BallEntry>& balls,
                             const std::set<std::pair<std::string, std::string>>& neighbours,
                             double minimumSize)
{
    std::vector<double> newRadius;
    std::vector<double> nonNeighbourTarget(balls.size(), std::numeric_limits<double>::infinity());
    for (auto& entry : balls)
        newRadius.push_back(entry.second->radius);
    for (std::size_t i = 0; i < balls.size(); ++i)
        for (std::size_t j = i + 1; j < balls.size(); ++j)
        {
            auto& ball0 = *balls[i].second;
            auto& ball1 = *balls[j].second;
            auto distance = (ball0.point - ball1.point).norm();
            if (distance >= ball0.radius + ball1.radius)
                continue;
            if (neighbours.count(std::minmax(ballKey(balls[i].first), ballKey(balls[j].first))))
            {
                // Beyond CGAL: a neighbour inside the other's ball is
                // hidden. CGAL's slowly varying sizing never makes one;
                // following a size function that varies fiftyfold along
                // a curve can, next to a large corner ball. Shrinking the
                // larger ball to their distance unhides the smaller and
                // keeps the two overlapping.
                auto larger = ball0.radius >= ball1.radius ? i : j;
                if (distance < std::max(ball0.radius, ball1.radius))
                    newRadius[larger] = std::min(newRadius[larger], distance);
                continue;
            }
            nonNeighbourTarget[i] = std::min(nonNeighbourTarget[i], distance / DISTANCE_DIVISOR);
            nonNeighbourTarget[j] = std::min(nonNeighbourTarget[j], distance / DISTANCE_DIVISOR);
        }

    bool changed = false;
    for (std::size_t i = 0; i < balls.size(); ++i)
    {
        auto& ball = *balls[i].second;
        auto radius = newRadius[i];
        if (!ball.atMinimumSize && nonNeighbourTarget[i] < minimumSize)
        {
            radius = minimumSize;
            ball.atMinimumSize = true;
        }
        else if (!ball.atMinimumSize)
            radius = std::min(radius, nonNeighbourTarget[i]);
        changed = changed || radius != ball.radius;
        ball.radius = radius;
    }
    return changed;
}

/// CGAL's refine_balls: shrink intersecting non-neighbours, repopulate gaps,
/// repeat. Returns the number of rounds that changed something.
int repairBalls(std::vector<Curve>& curves,
                const std::vector<CurveEnds>& ends,
                std::map<std::string, Ball>& corners,
                double minimumSize)
{
    for (int round = 0; round < MAXIMUM_REPAIR_ROUNDS; ++round)
    {
        auto balls = collectBalls(curves, corners);
        auto neighbours = neighbourKeys(curves, ends);
        bool shrunk = shrinkIntersectingBalls(balls, neighbours, minimumSize);

        bool repopulated = false;
        for (std::size_t curveIndex = 0; curveIndex < curves.size(); ++curveIndex)
            repopulated = repopulate(curves[curveIndex], ends[curveIndex].startCorner,
                                     ends[curveIndex].endCorner, corners) ||
                          repopulated;

        if (!shrunk && !repopulated)
            return round;
    }
    spdlog::warn("ProtectingBallPlacer: repair did not settle in {} rounds", MAXIMUM_REPAIR_ROUNDS);
    return MAXIMUM_REPAIR_ROUNDS;
}

// Beyond CGAL (OPE-218): two curves meeting at a corner with the same
// away-from-corner tangent direction (to within TANGENT_COSINE) are G1-
// tangent there -- a straight-to-arc or arc-to-arc transition with no
// turning angle to size a corner against. Their separation then grows only
// quadratically with arc length (gap(s) ~= |kappaA - kappaB| / 2 * s^2), so
// repairBalls' shrink-on-conflict never converges: each round's smaller
// radius places the next ball closer to the corner, where the quadratic gap
// has shrunk faster than the radius did. The fix is a floor, not a cap --
// size the corner ball so it already reaches past the point where two
// cornerSize-radius balls, one on each curve, stop conflicting.
constexpr double TANGENT_COSINE = 0.9848; // cos(10 degrees)

/// Unit tangent direction and curvature of `curve` at the end identified by
/// `atStart`, direction oriented away from that end's corner.
struct CornerTangent
{
    Vector3D direction;
    double curvature;
};

CornerTangent cornerTangentOf(const Curve& curve, bool atStart)
{
    auto arc = atStart ? 0.0 : curve.length;
    auto parameter = parameterAt(curve, arc);
    Vector3D direction = curve.edge->getTangent(parameter).normalized();
    if (!atStart)
        direction = -direction;
    return {direction, curve.edge->getCurvature(parameter)};
}

/// The largest corner radius any G1-tangent pair of curves at `cornerId`
/// demands (0 if none are tangent there). See the comment above.
double tangentCornerFloor(const std::string& cornerId,
                          const std::vector<Curve>& curves,
                          const std::vector<CurveEnds>& ends,
                          double cornerSize)
{
    std::vector<CornerTangent> tangents;
    for (std::size_t curveIndex = 0; curveIndex < curves.size(); ++curveIndex)
    {
        if (ends[curveIndex].startCorner == cornerId)
            tangents.push_back(cornerTangentOf(curves[curveIndex], true));
        if (ends[curveIndex].endCorner == cornerId)
            tangents.push_back(cornerTangentOf(curves[curveIndex], false));
    }

    double floor = 0.0;
    for (std::size_t i = 0; i < tangents.size(); ++i)
        for (std::size_t j = i + 1; j < tangents.size(); ++j)
        {
            if (tangents[i].direction.dot(tangents[j].direction) < TANGENT_COSINE)
                continue;
            // Smaller curvatureGap means slower separation, so a larger
            // floor -- including the curvatureGap == 0 limit (equal-radius
            // fillets forming a G2-tangent corner), where no finite radius
            // satisfies this second-order bound at all: IEEE-754 division
            // by zero gives +infinity here (cornerSize > 0), which the
            // nearest-corner cap on the call site then clamps down to the
            // best available radius rather than silently applying no floor.
            auto curvatureGap = std::abs(tangents[i].curvature - tangents[j].curvature);
            floor = std::max(floor, std::sqrt(4.0 * cornerSize / curvatureGap));
        }
    return floor;
}

/// Corner radius: the size at the corner (the smallest adjacent discretization
/// segment of its curves), raised to tangentCornerFloor when a tangent curve
/// pair demands more room (OPE-218), then capped at a third of the nearest
/// other corner and by the surfaces not containing it.
std::map<std::string, Ball> placeCorners(const std::vector<Curve>& curves,
                                         const std::vector<CurveEnds>& ends,
                                         const DiscretizationResult3D& discretization,
                                         const SurfaceCap& cap)
{
    std::map<std::string, double> size;
    for (std::size_t curveIndex = 0; curveIndex < curves.size(); ++curveIndex)
    {
        auto& breaks = curves[curveIndex].sizeBreaks;
        auto first = breaks[1] - breaks[0];
        auto last = breaks[breaks.size() - 1] - breaks[breaks.size() - 2];
        for (auto& [cornerId, value] : {std::make_pair(ends[curveIndex].startCorner, first),
                                        std::make_pair(ends[curveIndex].endCorner, last)})
        {
            auto found = size.find(cornerId);
            size[cornerId] = found == size.end() ? value : std::min(found->second, value);
        }
    }

    std::map<std::string, Ball> corners;
    for (auto& [cornerId, cornerSize] : size)
    {
        auto& point = discretization.points[discretization.cornerIdToPointIndexMap.at(cornerId)];
        double nearest = std::numeric_limits<double>::max();
        for (auto& [otherId, otherIndex] : discretization.cornerIdToPointIndexMap)
            if (otherId != cornerId)
                nearest = std::min(nearest, (discretization.points[otherIndex] - point).norm());
        auto floor = tangentCornerFloor(cornerId, curves, ends, cornerSize);
        auto radius = std::max(std::min({std::max(cornerSize, floor), nearest / 3.0, capAt(cap, point, {cornerId})}), cap.floor);
        corners[cornerId] = Ball{0.0, point, radius, cornerId};
    }
    return corners;
}

double boundingBoxDiagonal(const std::vector<Point3D>& points)
{
    if (points.empty()) return 0.0;
    Point3D minimum = points.front();
    Point3D maximum = points.front();
    for (auto& point : points)
    {
        minimum = minimum.cwiseMin(point);
        maximum = maximum.cwiseMax(point);
    }
    return (maximum - minimum).norm();
}

/// How many balls, corners included, the surface cap is holding down.
std::size_t countBallsAtCap(const std::vector<Curve>& curves, const std::map<std::string, Ball>& corners, const SurfaceCap& cap)
{
    auto atCap = [&](const Ball& ball, const std::vector<std::string>& cornerIds)
    { return ball.radius >= capAt(cap, ball.point, cornerIds) * (1.0 - AT_CAP_TOLERANCE); };

    std::size_t count = 0;
    for (auto& [cornerId, ball] : corners)
        count += atCap(ball, {cornerId}) ? 1 : 0;
    for (auto& curve : curves)
        for (auto& ball : curve.balls)
            count += atCap(ball, curve.cornerIds) ? 1 : 0;
    return count;
}

/// How many balls, corners included, repair stopped at their minimum size.
std::size_t countBallsAtMinimumSize(const std::vector<Curve>& curves, const std::map<std::string, Ball>& corners)
{
    std::size_t count = 0;
    for (auto& [cornerId, ball] : corners)
        count += ball.atMinimumSize ? 1 : 0;
    for (auto& curve : curves)
        count += std::ranges::count_if(curve.balls, [](const Ball& ball)
                                       { return ball.atMinimumSize; });
    return count;
}

/// The discretization restricted to what protection leaves alone: corners,
/// surface points, and the edges that are neither protected nor seam twins.
DiscretizationResult3D keepUnprotected(const DiscretizationResult3D& discretization,
                                       const std::set<std::string>& protectedEdges,
                                       const Topology3D::SeamCollection& seams)
{
    std::set<std::size_t> kept;
    for (auto& [cornerId, index] : discretization.cornerIdToPointIndexMap)
        kept.insert(index);
    for (auto& [surfaceId, indices] : discretization.surfaceIdToPointIndicesMap)
        kept.insert(indices.begin(), indices.end());
    for (auto& [edgeId, chain] : discretization.edgeIdToPointIndicesMap)
        if (!protectedEdges.count(edgeId) && !seams.isSeamTwin(edgeId))
            kept.insert(chain.begin(), chain.end());

    DiscretizationResult3D result;
    std::map<std::size_t, std::size_t> newIndexOf;
    for (auto index : kept)
    {
        newIndexOf[index] = result.points.size();
        result.points.push_back(discretization.points[index]);
        result.geometryIds.push_back(discretization.geometryIds[index]);
        result.edgeParameters.push_back(discretization.edgeParameters[index]);
    }
    for (auto& [cornerId, index] : discretization.cornerIdToPointIndexMap)
        result.cornerIdToPointIndexMap[cornerId] = newIndexOf.at(index);
    for (auto& [surfaceId, indices] : discretization.surfaceIdToPointIndicesMap)
        for (auto index : indices)
            result.surfaceIdToPointIndicesMap[surfaceId].push_back(newIndexOf.at(index));
    for (auto& [edgeId, chain] : discretization.edgeIdToPointIndicesMap)
        if (!protectedEdges.count(edgeId) && !seams.isSeamTwin(edgeId))
            for (auto index : chain)
                result.edgeIdToPointIndicesMap[edgeId].push_back(newIndexOf.at(index));
    return result;
}

/// Each seam twin gets its original edge's points in reverse.
void reverseSeamTwins(DiscretizationResult3D& result, const Topology3D::SeamCollection& seams)
{
    for (auto& twinId : seams.getSeamTwinEdgeIds())
    {
        auto original = result.edgeIdToPointIndicesMap.find(seams.getOriginalEdgeId(twinId));
        if (original != result.edgeIdToPointIndicesMap.end())
            result.edgeIdToPointIndicesMap[twinId] = {original->second.rbegin(), original->second.rend()};
    }
}

/// Rebuilds discretization: kept points (corners, surface points, degenerate
/// edges) first, then every protected curve's new interior points. Returns
/// the weights by new index.
std::unordered_map<std::size_t, double> rebuild(DiscretizationResult3D& discretization,
                                                const std::vector<Curve>& curves,
                                                const std::vector<CurveEnds>& ends,
                                                const std::map<std::string, Ball>& corners,
                                                const Topology3D::Topology3D& topology)
{
    std::set<std::string> protectedEdges;
    for (auto& curve : curves)
        protectedEdges.insert(curve.edgeId);
    auto& seams = topology.getSeamCollection();

    auto result = keepUnprotected(discretization, protectedEdges, seams);

    std::unordered_map<std::size_t, double> weights;
    for (auto& [cornerId, ball] : corners)
        weights[result.cornerIdToPointIndexMap.at(cornerId)] = ball.radius * ball.radius;
    for (std::size_t curveIndex = 0; curveIndex < curves.size(); ++curveIndex)
    {
        auto& chain = result.edgeIdToPointIndicesMap[curves[curveIndex].edgeId];
        chain.push_back(result.cornerIdToPointIndexMap.at(ends[curveIndex].startCorner));
        for (auto& ball : curves[curveIndex].balls)
        {
            weights[result.points.size()] = ball.radius * ball.radius;
            chain.push_back(result.points.size());
            result.points.push_back(ball.point);
            result.geometryIds.push_back({curves[curveIndex].edgeId});
            result.edgeParameters.push_back({parameterAt(curves[curveIndex], ball.arc)});
        }
        chain.push_back(result.cornerIdToPointIndexMap.at(ends[curveIndex].endCorner));
    }
    reverseSeamTwins(result, seams);
    discretization = std::move(result);
    return weights;
}

} // namespace

std::unordered_map<std::size_t, double> ProtectingBallPlacer::place(DiscretizationResult3D& discretization,
                                                                    const Topology3D::Topology3D& topology,
                                                                    const Geometry3D::GeometryCollection3D& geometry)
{
    NonIncidentSurfaceDistance surfaceDistance(geometry, topology);
    auto minimumSize = MINIMUM_CAP_FRACTION * boundingBoxDiagonal(discretization.points);
    SurfaceCap cap{&surfaceDistance, minimumSize};

    std::vector<Curve> curves;
    std::vector<CurveEnds> ends;
    for (auto& [edgeId, chain] : discretization.edgeIdToPointIndicesMap)
    {
        if (topology.getSeamCollection().isSeamTwin(edgeId))
            continue;
        auto edge = geometry.getEdge(edgeId);
        if (!edge || edge->isDegenerate() || chain.size() < 2)
            continue;
        auto& topologyEdge = topology.getEdge(edgeId);
        auto length = edge->getLength();
        curves.push_back({edgeId, edge, cap, {topologyEdge.getStartCornerId(), topologyEdge.getEndCornerId()}, length, sizeBreaksOf(chain, discretization.points, length), {}});
        ends.push_back({topologyEdge.getStartCornerId(), topologyEdge.getEndCornerId()});
    }

    auto corners = placeCorners(curves, ends, discretization, cap);
    for (std::size_t curveIndex = 0; curveIndex < curves.size(); ++curveIndex)
    {
        auto& curve = curves[curveIndex];
        auto closedLoop = ends[curveIndex].startCorner == ends[curveIndex].endCorner;
        curve.balls = insertBalls(curve, cornerBall(curve, ends[curveIndex].startCorner, 0.0, corners),
                                  cornerBall(curve, ends[curveIndex].endCorner, curve.length, corners), closedLoop);
    }
    auto rounds = repairBalls(curves, ends, corners, minimumSize);

    std::size_t curvePoints = 0;
    for (auto& curve : curves)
        curvePoints += curve.balls.size();
    spdlog::info("ProtectingBallPlacer: {} corners, {} curve points on {} curves after {} repair rounds, "
                 "{} balls at their surface cap, {} at their minimum size",
                 corners.size(), curvePoints, curves.size(), rounds, countBallsAtCap(curves, corners, cap),
                 countBallsAtMinimumSize(curves, corners));
    return rebuild(discretization, curves, ends, corners, topology);
}

} // namespace Meshing
