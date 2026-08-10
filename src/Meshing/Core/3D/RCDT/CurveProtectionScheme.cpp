#include "Meshing/Core/3D/RCDT/CurveProtectionScheme.h"

#include "spdlog/spdlog.h"
#include <algorithm>
#include <limits>

namespace Meshing
{

namespace
{

// See class comment, property 1: using the LONGER of a point's two adjacent
// segments (rather than the shorter, or an average) means that for any
// segment [a,b] of length L, both a's and b's radius are bounded below by
// INTERIOR_FACTOR * L specifically (L is always one of the two candidates
// in each endpoint's own max) -- so their sum is bounded below by
// 2*INTERIOR_FACTOR*L, unconditionally, regardless of how the *other*
// segment at either endpoint compares to L. INTERIOR_FACTOR > 0.5 makes
// that sum exceed L, guaranteeing overlap. Does not apply as-is to a
// segment with a corner at one end -- see CORNER_OVERLAP_SLACK below.
constexpr double INTERIOR_FACTOR = 0.6;

// A corner's ball is sized independently of the interior scheme, from the
// shortest first step of any incident curve -- kept below INTERIOR_FACTOR
// so it stays a "strong" (smaller) ball its curves' own interior radii
// shrink toward, never the reverse. Strictly less than 1: a corner shared
// by curves of very different scales sizes itself from the shortest one,
// which can be far below another incident curve's own first-step length,
// so that curve's first interior point cannot rely on the general
// INTERIOR_FACTOR formula alone to overlap this (possibly much smaller)
// corner ball -- see CORNER_OVERLAP_SLACK.
constexpr double CORNER_FACTOR = 0.3;

// A corner-adjacent segment [corner, firstInterior] needs
// radius(corner) + radius(firstInterior) > length, but radius(corner) can
// be arbitrarily small relative to this specific segment (see
// CORNER_FACTOR) if another incident curve at the same corner is much
// shorter. So the first interior point on each incident edge is sized
// directly against its OWN corner's already-computed radius instead of
// relying on the generic max-of-neighbors formula: radius(firstInterior) =
// CORNER_OVERLAP_SLACK * (length - radius(corner)). Since radius(corner) <
// length always (CORNER_FACTOR < 1 and the corner's radius is at most
// CORNER_FACTOR times this edge's own first step), (length -
// radius(corner)) > 0, and CORNER_OVERLAP_SLACK > 1 gives
// radius(corner) + radius(firstInterior)
//   = radius(corner) + CORNER_OVERLAP_SLACK * (length - radius(corner))
//   = CORNER_OVERLAP_SLACK * length - (CORNER_OVERLAP_SLACK - 1) * radius(corner)
//   > CORNER_OVERLAP_SLACK * length - (CORNER_OVERLAP_SLACK - 1) * length   [radius(corner) < length]
//   = length,
// unconditionally.
constexpr double CORNER_OVERLAP_SLACK = 1.1;

// See class comment, property 2: clamping every point's radius to at most
// DISJOINT_FACTOR times its distance to the nearest point belonging to a
// DIFFERENT edge (see "related" below) is sufficient to keep every
// unrelated pair's balls from overlapping. For any unrelated pair (x,y),
// each point's own nearest-unrelated distance is by definition <= dist(x,y)
// (y itself is *a* unrelated point to x, and vice versa), so
// radius(x) <= DISJOINT_FACTOR * dist(x,y) and
// radius(y) <= DISJOINT_FACTOR * dist(x,y) both hold, giving
// radius(x) + radius(y) <= 2*DISJOINT_FACTOR*dist(x,y) < dist(x,y) whenever
// DISJOINT_FACTOR < 0.5.
constexpr double DISJOINT_FACTOR = 0.45;

} // namespace

std::unordered_map<size_t, double> CurveProtectionScheme::computeWeights(
    const std::map<std::string, std::vector<size_t>>& edgeIdToPointIndicesMap,
    const std::unordered_set<size_t>& cornerPointIndices,
    const std::vector<Point3D>& points)
{
    std::unordered_set<size_t> featurePoints = cornerPointIndices;
    std::map<std::string, std::vector<double>> segmentLengthByEdge;

    // Which edges each point belongs to (a corner can belong to several; an
    // interior point belongs to exactly one) -- see "related" below.
    std::unordered_map<size_t, std::unordered_set<std::string>> pointToEdges;

    // Pass 1: feature-point/edge membership and per-edge segment lengths --
    // needed before corner radii (which pool across every incident edge)
    // can be computed.
    for (const auto& [edgeId, chain] : edgeIdToPointIndicesMap)
    {
        if (chain.size() < 2)
            continue;

        std::vector<double>& segmentLength = segmentLengthByEdge[edgeId];
        segmentLength.resize(chain.size() - 1);
        for (size_t k = 0; k + 1 < chain.size(); ++k)
        {
            segmentLength[k] = (points[chain[k]] - points[chain[k + 1]]).norm();
            featurePoints.insert(chain[k]);
            pointToEdges[chain[k]].insert(edgeId);
        }
        featurePoints.insert(chain.back());
        pointToEdges[chain.back()].insert(edgeId);
    }

    // Pass 2: corner radii, pooled across every incident edge (see
    // CORNER_FACTOR).
    std::unordered_map<size_t, double> cornerMinFirstStep;
    auto trackMin = [&cornerMinFirstStep](size_t cornerPoint, double step)
    {
        auto [it, inserted] = cornerMinFirstStep.try_emplace(cornerPoint, step);
        if (!inserted)
            it->second = std::min(it->second, step);
    };
    for (const auto& [edgeId, chain] : edgeIdToPointIndicesMap)
    {
        if (chain.size() < 2)
            continue;
        const auto& segmentLength = segmentLengthByEdge.at(edgeId);
        trackMin(chain.front(), segmentLength.front());
        trackMin(chain.back(), segmentLength.back());
    }

    std::unordered_map<size_t, double> localRadius;
    for (const auto& [cornerPoint, minFirstStep] : cornerMinFirstStep)
        localRadius[cornerPoint] = CORNER_FACTOR * minFirstStep;

    // Pass 3: interior-point radii, now that every corner's radius is
    // known. The general max-of-neighbors formula (property 1) applies
    // except where an interior point is adjacent to a corner, where it
    // must additionally satisfy CORNER_OVERLAP_SLACK against that specific
    // corner's already-fixed radius (see CORNER_OVERLAP_SLACK) -- a chain
    // with exactly one interior point is adjacent to a corner on both
    // sides at once and must satisfy both.
    for (const auto& [edgeId, chain] : edgeIdToPointIndicesMap)
    {
        if (chain.size() < 3)
            continue; // no interior points
        const auto& segmentLength = segmentLengthByEdge.at(edgeId);

        for (size_t i = 1; i + 1 < chain.size(); ++i)
        {
            double radius = INTERIOR_FACTOR * std::max(segmentLength[i - 1], segmentLength[i]);
            if (i == 1)
            {
                const double cornerRadius = localRadius.at(chain.front());
                radius = std::max(radius, CORNER_OVERLAP_SLACK * (segmentLength[i - 1] - cornerRadius));
            }
            if (i + 2 == chain.size())
            {
                const double cornerRadius = localRadius.at(chain.back());
                radius = std::max(radius, CORNER_OVERLAP_SLACK * (segmentLength[i] - cornerRadius));
            }
            localRadius[chain[i]] = radius;
        }
    }

    // Property 2: clamp every feature point's radius against the nearest
    // point belonging to none of the same edges (a point on the SAME curve,
    // however many hops away, is the same 1D feature -- property 1 alone
    // governs it; only a genuinely different edge counts as "unrelated"
    // here). A corner's related set spans every edge it's incident to.
    // Brute-force O(n^2) over feature points -- fine at the boundary-
    // discretization point counts RCDT deals with today; revisit with a
    // spatial index if profiling ever shows this dominating (see e.g. the
    // SurfaceTessellation/RCDTRefiner incrementalization history for the
    // project's general "correct first, optimize when it's an actual
    // bottleneck" pattern).
    const std::vector<size_t> orderedFeaturePoints(featurePoints.begin(), featurePoints.end());
    std::unordered_map<size_t, double> finalRadius;
    for (size_t p : orderedFeaturePoints)
    {
        double nearestUnrelated = std::numeric_limits<double>::infinity();
        const auto& ownEdges = pointToEdges[p];
        auto sharesAnEdge = [&ownEdges, &pointToEdges](size_t q)
        {
            const auto& otherEdges = pointToEdges[q];
            return std::any_of(ownEdges.begin(), ownEdges.end(),
                               [&otherEdges](const std::string& edgeId) { return otherEdges.contains(edgeId); });
        };
        for (size_t q : orderedFeaturePoints)
        {
            if (q == p || sharesAnEdge(q))
                continue;
            nearestUnrelated = std::min(nearestUnrelated, (points[p] - points[q]).norm());
        }

        const double local = localRadius.contains(p) ? localRadius.at(p) : 0.0;
        const double clamp = DISJOINT_FACTOR * nearestUnrelated;
        finalRadius[p] = std::min(local, clamp);
    }

    // Verify property 1 survived the property-2 clamp; a violation means an
    // unrelated feature passes close enough to this curve, relative to its
    // own local sampling, that no single-pass sizing can satisfy both
    // properties at once -- flagged rather than silently producing a gap in
    // the crease protection (see class comment).
    for (const auto& [edgeId, chain] : edgeIdToPointIndicesMap)
    {
        for (size_t k = 0; k + 1 < chain.size(); ++k)
        {
            const double distance = (points[chain[k]] - points[chain[k + 1]]).norm();
            if (finalRadius[chain[k]] + finalRadius[chain[k + 1]] <= distance)
            {
                spdlog::warn("CurveProtectionScheme::computeWeights: protecting balls for edge '{}' do not "
                            "overlap between points {} and {} (radii {} + {} <= segment length {}) -- an "
                            "unrelated feature is too close to this curve for its current sampling density",
                            edgeId, chain[k], chain[k + 1], finalRadius[chain[k]], finalRadius[chain[k + 1]],
                            distance);
            }
        }
    }

    std::unordered_map<size_t, double> weights;
    weights.reserve(finalRadius.size());
    for (const auto& [point, radius] : finalRadius)
        weights[point] = radius * radius;
    return weights;
}

} // namespace Meshing
