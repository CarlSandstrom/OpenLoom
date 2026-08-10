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
// radius(corner) + radius(firstInterior) > length, sized against the
// corner's FINAL radius (after property 2's clamp -- see
// computeWeights()'s corners-before-interior ordering) rather than its
// pre-clamp local value: radius(firstInterior) =
// CORNER_OVERLAP_SLACK * (length - radius(corner)). Since radius(corner) <
// length always for the pre-clamp value (CORNER_FACTOR < 1 and the
// corner's local radius is at most CORNER_FACTOR times this edge's own
// first step) and clamping only ever shrinks it further, (length -
// radius(corner)) > 0 still holds for the final value, and
// CORNER_OVERLAP_SLACK > 1 gives
// radius(corner) + radius(firstInterior)
//   = radius(corner) + CORNER_OVERLAP_SLACK * (length - radius(corner))
//   = CORNER_OVERLAP_SLACK * length - (CORNER_OVERLAP_SLACK - 1) * radius(corner)
//   > CORNER_OVERLAP_SLACK * length - (CORNER_OVERLAP_SLACK - 1) * length   [radius(corner) < length]
//   = length,
// unconditionally -- *provided* the interior point's own disjointness
// clamp (property 2, applied after this) doesn't then shrink it back
// below what this guarantees. See class comment for when that can still
// happen (a genuine local-feature-size conflict, not a sequencing bug).
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
// DISJOINT_FACTOR < 0.5. This clamp is purely positional (a function of
// point positions only, never of any point's radius), so it can be
// evaluated for any point independent of processing order.
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

    // Pass 2: corner LOCAL radii, pooled across every incident edge (see
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

    std::unordered_map<size_t, double> cornerLocalRadius;
    for (const auto& [cornerPoint, minFirstStep] : cornerMinFirstStep)
        cornerLocalRadius[cornerPoint] = CORNER_FACTOR * minFirstStep;

    // Property 2's clamp (see DISJOINT_FACTOR) is purely positional, so it
    // can be applied to corners before interior radii are even computed --
    // and MUST be, so pass 3's CORNER_OVERLAP_SLACK compensation sizes
    // against each corner's real, final radius rather than a pre-clamp
    // value a later clamp could shrink out from under it (that ordering
    // bug -- computing the corner-adjacent interior compensation against
    // localRadius instead of the corner's post-clamp radius -- is what
    // originally broke RCDTMesherTorusTest: the compensation looked correct
    // relative to a corner radius that no longer existed by the time both
    // were compared).
    const std::vector<size_t> orderedFeaturePoints(featurePoints.begin(), featurePoints.end());
    auto nearestUnrelatedDistance = [&](size_t p)
    {
        double nearest = std::numeric_limits<double>::infinity();
        const auto& ownEdges = pointToEdges[p];
        for (size_t q : orderedFeaturePoints)
        {
            if (q == p)
                continue;
            const auto& otherEdges = pointToEdges[q];
            const bool related = std::any_of(ownEdges.begin(), ownEdges.end(), [&otherEdges](const std::string& edgeId)
                                             { return otherEdges.contains(edgeId); });
            if (related)
                continue;
            nearest = std::min(nearest, (points[p] - points[q]).norm());
        }
        return nearest;
    };

    std::unordered_map<size_t, double> finalRadius;
    for (const auto& [cornerPoint, localRadius] : cornerLocalRadius)
        finalRadius[cornerPoint] = std::min(localRadius, DISJOINT_FACTOR * nearestUnrelatedDistance(cornerPoint));

    // Pass 3: interior-point LOCAL radii, now that every corner's FINAL
    // radius is known. The general max-of-neighbors formula (property 1)
    // applies except where an interior point is adjacent to a corner,
    // where it must additionally satisfy CORNER_OVERLAP_SLACK against that
    // corner's final radius -- a chain with exactly one interior point is
    // adjacent to a corner on both sides at once and must satisfy both.
    std::unordered_map<size_t, double> interiorLocalRadius;
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
                const double cornerRadius = finalRadius.at(chain.front());
                radius = std::max(radius, CORNER_OVERLAP_SLACK * (segmentLength[i - 1] - cornerRadius));
            }
            if (i + 2 == chain.size())
            {
                const double cornerRadius = finalRadius.at(chain.back());
                radius = std::max(radius, CORNER_OVERLAP_SLACK * (segmentLength[i] - cornerRadius));
            }
            interiorLocalRadius[chain[i]] = radius;
        }
    }

    // Clamp interior points. Brute-force O(n^2) over feature points -- fine
    // at the boundary-discretization point counts RCDT deals with today;
    // revisit with a spatial index if profiling ever shows this dominating
    // (see e.g. the SurfaceTessellation/RCDTRefiner incrementalization
    // history for the project's general "correct first, optimize when it's
    // an actual bottleneck" pattern).
    for (const auto& [interiorPoint, localRadius] : interiorLocalRadius)
        finalRadius[interiorPoint] = std::min(localRadius, DISJOINT_FACTOR * nearestUnrelatedDistance(interiorPoint));

    // A feature point with neither a corner nor an interior local radius
    // (e.g. an isolated corner with no incident edges) gets no protection.
    for (size_t p : orderedFeaturePoints)
        finalRadius.try_emplace(p, 0.0);

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
