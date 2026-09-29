#include "Meshing/Core/3D/RCDT/ProtectingBallPlacer.h"

#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/IEdge3D.h"
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

/// One protecting ball on a curve, by arc length from the curve's start.
struct Ball
{
    double arc;
    Point3D point;
    double radius;
    std::string cornerId; // empty for a ball inside the curve
};

/// A protected curve: its geometry, the size function read from the existing
/// discretization's spacing, and its balls in order from start to end.
struct Curve
{
    std::string edgeId;
    const Geometry3D::IEdge3D* edge;
    double length;
    std::vector<double> sizeBreaks; // arc positions of the discretization's points
    std::vector<Ball> balls;
};

double sizeAt(const Curve& curve, double arc)
{
    const auto& breaks = curve.sizeBreaks;
    const auto upper = std::upper_bound(breaks.begin(), breaks.end(), arc);
    const std::size_t segment = std::clamp<std::size_t>(upper - breaks.begin(), 1, breaks.size() - 1);
    return breaks[segment] - breaks[segment - 1];
}

Point3D pointAt(const Curve& curve, double arc)
{
    const auto [start, end] = curve.edge->getParameterBounds();
    return curve.edge->getPoint(curve.edge->getParameterAtArcLengthFraction(start, end, arc / curve.length));
}

double parameterAt(const Curve& curve, double arc)
{
    const auto [start, end] = curve.edge->getParameterBounds();
    return curve.edge->getParameterAtArcLengthFraction(start, end, arc / curve.length);
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
    const double chordLength = breaks.back();
    for (double& value : breaks)
        value *= length / chordLength;
    return breaks;
}

/// CGAL's insert_balls: the balls strictly between a and b (a.arc < b.arc),
/// in increasing arc order.
std::vector<Ball> insertBalls(const Curve& curve, const Ball& a, const Ball& b, bool closedLoop)
{
    const double d = b.arc - a.arc;
    if (d <= 0.0)
        return {};
    const bool forward = a.radius <= b.radius;
    const double sp = std::min(a.radius, b.radius);
    const double sq = std::max(a.radius, b.radius);
    const double estimate = std::floor(2.0 * (d - sq) / (sp + sq) + 0.5);
    std::size_t n = estimate > 0.0 ? static_cast<std::size_t>(estimate) : 0;

    // Place a ball at the midpoint with the local size first, so the size
    // function is followed instead of interpolated between the ends: CGAL
    // does so for a long run; it must also happen wherever interpolation
    // would oversize the middle. CGAL's sizing fields vary slowly, but a
    // spacing read off an angle-based discretization does not -- on the dense
    // saddle's end parabolas it falls from 1.33 at the corners to 0.028 at
    // the apex, and interpolating between the corners put balls of radius
    // 0.8 there (OPE-186). The midpoint must lie outside both end balls, or
    // its ball would be hidden.
    const double middleArc = a.arc + d / 2.0;
    const Ball middle{middleArc, pointAt(curve, middleArc), sizeAt(curve, middleArc), {}};
    const bool oversizedMiddle = n >= 1 && middle.radius < (sp + sq) / 2.0 &&
                                 (middle.point - a.point).norm() > a.radius &&
                                 (middle.point - b.point).norm() > b.radius;
    if (n >= REEVALUATE_SIZE_ABOVE || oversizedMiddle)
    {
        auto balls = insertBalls(curve, a, middle, false);
        balls.push_back(middle);
        const auto second = insertBalls(curve, middle, b, false);
        balls.insert(balls.end(), second.begin(), second.end());
        return balls;
    }

    const double r = (sq - sp) / static_cast<double>(n + 1);
    const double covered = sp * static_cast<double>(n + 1) + static_cast<double>((n + 1) * (n + 2)) / 2.0 * r;
    const double fraction = d / covered;
    double step = sp + r;
    double normalizedStep = fraction * step;
    double distance = normalizedStep;
    if (n == 0 && d >= sp + sq)
    {
        n = 1;
        step = sp + (d - sp - sq) / 2.0;
        distance = step;
        normalizedStep = step;
    }
    else if (closedLoop && n == 1)
    {
        n = 2;
        step = d / 3.0;
        distance = step;
        normalizedStep = step;
    }

    std::vector<Ball> balls;
    for (std::size_t i = 1; i <= n; ++i)
    {
        const double arc = forward ? a.arc + distance : b.arc - distance;
        const double radius = std::min(normalizedStep, sp + distance / d * (sq - sp));
        balls.push_back({arc, pointAt(curve, arc), radius, {}});
        if (r > 0.0)
        {
            step += r;
            normalizedStep = fraction * step;
        }
        distance += normalizedStep;
    }
    if (!forward)
        std::reverse(balls.begin(), balls.end());
    return balls;
}

/// The ball at each end of a curve: its corner's.
Ball cornerBall(const std::string& cornerId, double arc, const std::map<std::string, Ball>& corners)
{
    Ball ball = corners.at(cornerId);
    ball.arc = arc;
    return ball;
}

/// The curve's balls end to end, corners included.
std::vector<Ball> withCorners(const Curve& curve,
                              const std::string& startCorner,
                              const std::string& endCorner,
                              const std::map<std::string, Ball>& corners)
{
    std::vector<Ball> all{cornerBall(startCorner, 0.0, corners)};
    all.insert(all.end(), curve.balls.begin(), curve.balls.end());
    all.push_back(cornerBall(endCorner, curve.length, corners));
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
    const auto all = withCorners(curve, startCorner, endCorner, corners);
    const bool closedLoop = startCorner == endCorner;
    std::vector<Ball> interior;
    bool added = false;
    for (std::size_t i = 0; i + 1 < all.size(); ++i)
    {
        if (i > 0)
            interior.push_back(all[i]);
        if (all[i + 1].arc - all[i].arc <= all[i].radius + all[i + 1].radius)
            continue;
        const auto filler = insertBalls(curve, all[i], all[i + 1], closedLoop && all.size() == 2);
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

/// CGAL's refine_balls: shrink intersecting non-neighbours, repopulate gaps,
/// repeat. Returns the number of rounds that changed something.
int repairBalls(std::vector<Curve>& curves, const std::vector<CurveEnds>& ends, std::map<std::string, Ball>& corners)
{
    for (int round = 0; round < MAXIMUM_REPAIR_ROUNDS; ++round)
    {
        std::vector<std::pair<BallReference, Ball*>> balls;
        for (auto& [cornerId, ball] : corners)
            balls.push_back({{cornerId, 0, 0}, &ball});
        for (std::size_t c = 0; c < curves.size(); ++c)
            for (std::size_t i = 0; i < curves[c].balls.size(); ++i)
                balls.push_back({{"", c, i}, &curves[c].balls[i]});

        // Neighbours: consecutive along a curve, corners included.
        const auto key = [](const BallReference& reference)
        {
            return reference.cornerId.empty() ? "c" + std::to_string(reference.curve) + ":" +
                                                    std::to_string(reference.index) :
                                                "v" + reference.cornerId;
        };
        std::set<std::pair<std::string, std::string>> neighbours;
        for (std::size_t c = 0; c < curves.size(); ++c)
        {
            std::vector<std::string> chain{"v" + ends[c].startCorner};
            for (std::size_t i = 0; i < curves[c].balls.size(); ++i)
                chain.push_back(key({"", c, i}));
            chain.push_back("v" + ends[c].endCorner);
            for (std::size_t i = 0; i + 1 < chain.size(); ++i)
                neighbours.insert(std::minmax(chain[i], chain[i + 1]));
        }

        std::vector<double> newRadius;
        for (const auto& entry : balls)
            newRadius.push_back(entry.second->radius);
        bool shrunk = false;
        for (std::size_t i = 0; i < balls.size(); ++i)
            for (std::size_t j = i + 1; j < balls.size(); ++j)
            {
                const Ball& a = *balls[i].second;
                const Ball& b = *balls[j].second;
                const double distance = (a.point - b.point).norm();
                if (distance >= a.radius + b.radius)
                    continue;
                if (neighbours.count(std::minmax(key(balls[i].first), key(balls[j].first))))
                {
                    // Beyond CGAL: a neighbour inside the other's ball is
                    // hidden. CGAL's slowly varying sizing never makes one;
                    // following a size function that varies fiftyfold along
                    // a curve can, next to a large corner ball. Shrinking the
                    // larger ball to their distance unhides the smaller and
                    // keeps the two overlapping.
                    const std::size_t larger = a.radius >= b.radius ? i : j;
                    if (distance < std::max(a.radius, b.radius))
                    {
                        newRadius[larger] = std::min(newRadius[larger], distance);
                        shrunk = true;
                    }
                    continue;
                }
                newRadius[i] = std::min(newRadius[i], distance / DISTANCE_DIVISOR);
                newRadius[j] = std::min(newRadius[j], distance / DISTANCE_DIVISOR);
                shrunk = true;
            }
        for (std::size_t i = 0; i < balls.size(); ++i)
            balls[i].second->radius = newRadius[i];

        bool repopulated = false;
        for (std::size_t c = 0; c < curves.size(); ++c)
            repopulated = repopulate(curves[c], ends[c].startCorner, ends[c].endCorner, corners) || repopulated;

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
    const double arc = atStart ? 0.0 : curve.length;
    const double t = parameterAt(curve, arc);
    Vector3D direction = curve.edge->getTangent(t).normalized();
    if (!atStart)
        direction = -direction;
    return {direction, curve.edge->getCurvature(t)};
}

/// The largest corner radius any G1-tangent pair of curves at `cornerId`
/// demands (0 if none are tangent there). See the comment above.
double tangentCornerFloor(const std::string& cornerId,
                          const std::vector<Curve>& curves,
                          const std::vector<CurveEnds>& ends,
                          double cornerSize)
{
    std::vector<CornerTangent> tangents;
    for (std::size_t c = 0; c < curves.size(); ++c)
    {
        if (ends[c].startCorner == cornerId)
            tangents.push_back(cornerTangentOf(curves[c], true));
        if (ends[c].endCorner == cornerId)
            tangents.push_back(cornerTangentOf(curves[c], false));
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
            const double curvatureGap = std::abs(tangents[i].curvature - tangents[j].curvature);
            floor = std::max(floor, std::sqrt(4.0 * cornerSize / curvatureGap));
        }
    return floor;
}

/// Corner radius: the size at the corner (the smallest adjacent discretization
/// segment of its curves), capped at a third of the nearest other corner, and
/// raised to tangentCornerFloor when a tangent curve pair demands more room
/// (OPE-218).
std::map<std::string, Ball> placeCorners(const std::vector<Curve>& curves,
                                         const std::vector<CurveEnds>& ends,
                                         const DiscretizationResult3D& discretization)
{
    std::map<std::string, double> size;
    for (std::size_t c = 0; c < curves.size(); ++c)
    {
        const auto& breaks = curves[c].sizeBreaks;
        const double first = breaks[1] - breaks[0];
        const double last = breaks[breaks.size() - 1] - breaks[breaks.size() - 2];
        for (const auto& [cornerId, value] : {std::make_pair(ends[c].startCorner, first),
                                              std::make_pair(ends[c].endCorner, last)})
        {
            const auto found = size.find(cornerId);
            size[cornerId] = found == size.end() ? value : std::min(found->second, value);
        }
    }

    std::map<std::string, Ball> corners;
    for (const auto& [cornerId, cornerSize] : size)
    {
        const Point3D& point = discretization.points[discretization.cornerIdToPointIndexMap.at(cornerId)];
        double nearest = std::numeric_limits<double>::max();
        for (const auto& [otherId, otherIndex] : discretization.cornerIdToPointIndexMap)
            if (otherId != cornerId)
                nearest = std::min(nearest, (discretization.points[otherIndex] - point).norm());
        const double floor = tangentCornerFloor(cornerId, curves, ends, cornerSize);
        corners[cornerId] = Ball{0.0, point, std::min(std::max(cornerSize, floor), nearest / 3.0), cornerId};
    }
    return corners;
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
    for (const auto& curve : curves)
        protectedEdges.insert(curve.edgeId);
    const auto& seams = topology.getSeamCollection();

    std::set<std::size_t> kept;
    for (const auto& [cornerId, index] : discretization.cornerIdToPointIndexMap)
        kept.insert(index);
    for (const auto& [surfaceId, indices] : discretization.surfaceIdToPointIndicesMap)
        kept.insert(indices.begin(), indices.end());
    for (const auto& [edgeId, chain] : discretization.edgeIdToPointIndicesMap)
        if (!protectedEdges.count(edgeId) && !seams.isSeamTwin(edgeId))
            kept.insert(chain.begin(), chain.end());

    DiscretizationResult3D result;
    std::map<std::size_t, std::size_t> newIndexOf;
    for (const std::size_t index : kept)
    {
        newIndexOf[index] = result.points.size();
        result.points.push_back(discretization.points[index]);
        result.geometryIds.push_back(discretization.geometryIds[index]);
        result.edgeParameters.push_back(discretization.edgeParameters[index]);
    }
    for (const auto& [cornerId, index] : discretization.cornerIdToPointIndexMap)
        result.cornerIdToPointIndexMap[cornerId] = newIndexOf.at(index);
    for (const auto& [surfaceId, indices] : discretization.surfaceIdToPointIndicesMap)
        for (const std::size_t index : indices)
            result.surfaceIdToPointIndicesMap[surfaceId].push_back(newIndexOf.at(index));
    for (const auto& [edgeId, chain] : discretization.edgeIdToPointIndicesMap)
        if (!protectedEdges.count(edgeId) && !seams.isSeamTwin(edgeId))
            for (const std::size_t index : chain)
                result.edgeIdToPointIndicesMap[edgeId].push_back(newIndexOf.at(index));

    std::unordered_map<std::size_t, double> weights;
    for (const auto& [cornerId, ball] : corners)
        weights[result.cornerIdToPointIndexMap.at(cornerId)] = ball.radius * ball.radius;
    for (std::size_t c = 0; c < curves.size(); ++c)
    {
        auto& chain = result.edgeIdToPointIndicesMap[curves[c].edgeId];
        chain.push_back(result.cornerIdToPointIndexMap.at(ends[c].startCorner));
        for (const Ball& ball : curves[c].balls)
        {
            weights[result.points.size()] = ball.radius * ball.radius;
            chain.push_back(result.points.size());
            result.points.push_back(ball.point);
            result.geometryIds.push_back({curves[c].edgeId});
            result.edgeParameters.push_back({parameterAt(curves[c], ball.arc)});
        }
        chain.push_back(result.cornerIdToPointIndexMap.at(ends[c].endCorner));
    }
    for (const auto& twinId : seams.getSeamTwinEdgeIds())
    {
        const auto original = result.edgeIdToPointIndicesMap.find(seams.getOriginalEdgeId(twinId));
        if (original != result.edgeIdToPointIndicesMap.end())
            result.edgeIdToPointIndicesMap[twinId] = {original->second.rbegin(), original->second.rend()};
    }
    discretization = std::move(result);
    return weights;
}

} // namespace

std::unordered_map<std::size_t, double> ProtectingBallPlacer::place(DiscretizationResult3D& discretization,
                                                                    const Topology3D::Topology3D& topology,
                                                                    const Geometry3D::GeometryCollection3D& geometry)
{
    std::vector<Curve> curves;
    std::vector<CurveEnds> ends;
    for (const auto& [edgeId, chain] : discretization.edgeIdToPointIndicesMap)
    {
        if (topology.getSeamCollection().isSeamTwin(edgeId))
            continue;
        const Geometry3D::IEdge3D* edge = geometry.getEdge(edgeId);
        if (!edge || edge->isDegenerate() || chain.size() < 2)
            continue;
        const auto& topologyEdge = topology.getEdge(edgeId);
        const double length = edge->getLength();
        curves.push_back({edgeId, edge, length, sizeBreaksOf(chain, discretization.points, length), {}});
        ends.push_back({topologyEdge.getStartCornerId(), topologyEdge.getEndCornerId()});
    }

    auto corners = placeCorners(curves, ends, discretization);
    for (std::size_t c = 0; c < curves.size(); ++c)
    {
        const bool closedLoop = ends[c].startCorner == ends[c].endCorner;
        curves[c].balls = insertBalls(curves[c], cornerBall(ends[c].startCorner, 0.0, corners),
                                      cornerBall(ends[c].endCorner, curves[c].length, corners), closedLoop);
    }
    const int rounds = repairBalls(curves, ends, corners);

    std::size_t curvePoints = 0;
    for (const auto& curve : curves)
        curvePoints += curve.balls.size();
    spdlog::info("ProtectingBallPlacer: {} corners, {} curve points on {} curves after {} repair rounds",
                 corners.size(), curvePoints, curves.size(), rounds);
    return rebuild(discretization, curves, ends, corners, topology);
}

} // namespace Meshing
