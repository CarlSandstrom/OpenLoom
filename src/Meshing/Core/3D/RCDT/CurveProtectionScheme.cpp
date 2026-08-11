#include "Meshing/Core/3D/RCDT/CurveProtectionScheme.h"

#include "spdlog/spdlog.h"
#include <algorithm>
#include <cmath>
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
// corner ball -- see CORNER_OVERLAP_SLACK and CORNER_DILUTION_THRESHOLD.
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
// unconditionally -- *provided* radius(corner) is at least CORNER_FACTOR
// times THIS edge's own first step (see CORNER_DILUTION_THRESHOLD for when
// it isn't) and the interior point's own disjointness clamp (property 2,
// applied after this) doesn't then shrink it back below what this
// guarantees. See class comment for when that can still happen (a genuine
// local-feature-size conflict, not a sequencing bug).
constexpr double CORNER_OVERLAP_SLACK = 1.1;

// See class comment, property 2: clamping every point's radius to at most
// DISJOINT_FACTOR times its distance to the nearest point belonging to a
// DIFFERENT connected component of the curve network (see "related" below)
// is sufficient to keep every unrelated pair's balls from overlapping. For
// any unrelated pair (x,y), each point's own nearest-unrelated distance is
// by definition <= dist(x,y) (y itself is *a* unrelated point to x, and
// vice versa), so radius(x) <= DISJOINT_FACTOR * dist(x,y) and
// radius(y) <= DISJOINT_FACTOR * dist(x,y) both hold, giving
// radius(x) + radius(y) <= 2*DISJOINT_FACTOR*dist(x,y) < dist(x,y) whenever
// DISJOINT_FACTOR < 0.5. This clamp is purely positional (a function of
// point positions only, never of any point's radius), so it can be
// evaluated for any point independent of processing order. The same clamp
// also applies against the nearest point outside the curve network
// entirely (see nearestUnrelatedDistance() in computeWeights()) -- there
// the bound is even more comfortable, since such a point has no ball of its
// own to sum against: radius(x) <= DISJOINT_FACTOR * dist(x,q) < dist(x,q)
// unconditionally for DISJOINT_FACTOR < 1.
constexpr double DISJOINT_FACTOR = 0.45;

// A corner-adjacent segment [corner, firstInterior] needs
// radius(corner) + radius(firstInterior) > length. Bridging that in one
// jump -- radius(firstInterior) = CORNER_OVERLAP_SLACK * (length -
// radius(corner)) -- is provably safe (see CORNER_OVERLAP_SLACK) only while
// radius(corner) is itself proportionate to THIS edge's own length, i.e.
// this edge is (close to) the corner's own shortest incident edge, the one
// that actually set radius(corner) via CORNER_FACTOR. When a corner is
// shared with a much more finely-sampled sibling curve, CORNER_FACTOR sizes
// the corner from that sibling instead (see CORNER_FACTOR), starving this
// edge's compensation term: (length - radius(corner)) approaches the full,
// uncompensated length, so the "one jump" would inflate firstInterior's
// ball far past this edge's own local scale, potentially swallowing nearby
// geometry unrelated to the size mismatch that produced it (confirmed on
// RCDTMesherTorusTest's periodic seams, OPE-176). This threshold detects
// that dilution directly from the pre-clamp, cross-curve-pooled
// cornerMinFirstStep (see computeWeights()): compensation is only trusted
// while cornerMinFirstStep is at least this fraction of THIS edge's own
// first step -- i.e. this edge is close enough to being the one that set
// the corner's radius, not just borrowing a much finer sibling's. (The
// CORNER_FACTOR that would otherwise scale both sides of that ratio cancels
// out, so the comparison is done directly on the first-step lengths.)
// Below the threshold, the segment is left at its natural (uncompensated)
// radius and, if that leaves it unresolved, is closed by
// CurveProtectionSubdivider inserting points instead -- the same "insert
// points rather than inflate one ball" resolution the class comment
// describes for property 1/2 conflicts generally.
constexpr double CORNER_DILUTION_THRESHOLD = 0.5;

// Union-find over edge IDs, standard iterative find with path compression.
std::string find(std::unordered_map<std::string, std::string>& parent, const std::string& edgeId)
{
    std::string root = edgeId;
    while (parent[root] != root)
        root = parent[root];
    std::string current = edgeId;
    while (parent[current] != root)
    {
        std::string next = parent[current];
        parent[current] = root;
        current = next;
    }
    return root;
}

void unite(std::unordered_map<std::string, std::string>& parent, const std::string& a, const std::string& b)
{
    const std::string rootA = find(parent, a);
    const std::string rootB = find(parent, b);
    if (rootA != rootB)
        parent[rootA] = rootB;
}

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

    // Which connected component of the curve network each edge belongs to:
    // two edges are joined whenever they share a corner, transitively. A
    // corner shared by several curves (e.g. a torus's two periodic seams,
    // both closing at the same vertex) makes all of them -- and every point
    // on them -- part of ONE feature, not genuinely different "unrelated"
    // features property 2 needs to keep apart. Without this (an earlier
    // version related two points only if they shared the exact same single
    // edge), two curves meeting at a corner fought each other: subdividing
    // one curve's corner-adjacent segment brings its new point physically
    // closer to the OTHER curve's own corner-adjacent points (both
    // approach the same physical corner), which registered as an unrelated
    // proximity needing an even smaller clamp, needing another split,
    // bringing them closer still -- a runaway loop with no fixed point,
    // confirmed on RCDTMesherTorusTest (its two periodic seams share their
    // one corner).
    std::unordered_map<std::string, std::string> edgeComponentParent;
    for (const auto& [edgeId, chain] : edgeIdToPointIndicesMap)
        edgeComponentParent.try_emplace(edgeId, edgeId);
    for (const auto& [point, edges] : pointToEdges)
    {
        if (edges.size() < 2)
            continue;
        auto it = edges.begin();
        const std::string& first = *it;
        for (++it; it != edges.end(); ++it)
            unite(edgeComponentParent, first, *it);
    }
    auto sameComponent = [&edgeComponentParent](const std::string& a, const std::string& b)
    { return find(edgeComponentParent, a) == find(edgeComponentParent, b); };

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
        // Any one of p's edges suffices: union-find already merged every
        // edge incident to the same corner as p into one component, so
        // checking a single representative against a single representative
        // of q is equivalent to checking all pairs.
        const std::string& ownEdge = *pointToEdges[p].begin();
        for (size_t q : orderedFeaturePoints)
        {
            if (q == p)
                continue;
            if (sameComponent(ownEdge, *pointToEdges[q].begin()))
                continue;
            nearest = std::min(nearest, (points[p] - points[q]).norm());
        }

        // A point outside the curve network entirely (not in featurePoints
        // at all -- typically a face-interior sample, see
        // BoundaryDiscretizer3D) can never be chain-adjacent or
        // same-component with p, so it's unconditionally unrelated too.
        // Without this, a coarsely-sampled straight edge's natural or
        // corner-compensated radius (property 1 alone, sized purely from
        // curve-network topology) can legitimately come out large enough to
        // swallow a nearby face-interior point the disjointness clamp above
        // never even looks at -- confirmed on SaddleSurfaceMesh's flat
        // bottom face, OPE-176. Same O(n) caveat as the loop above.
        for (size_t q = 0; q < points.size(); ++q)
        {
            if (featurePoints.count(q))
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
    // applies except where an interior point is adjacent to a corner whose
    // radius isn't diluted by a finer sibling curve (see
    // CORNER_DILUTION_THRESHOLD), where it must additionally satisfy
    // CORNER_OVERLAP_SLACK against that corner's final radius -- a chain
    // with exactly one interior point is adjacent to a corner on both sides
    // at once and must satisfy both independently.
    std::unordered_map<size_t, double> interiorLocalRadius;
    for (const auto& [edgeId, chain] : edgeIdToPointIndicesMap)
    {
        if (chain.size() < 3)
            continue; // no interior points
        const auto& segmentLength = segmentLengthByEdge.at(edgeId);

        for (size_t i = 1; i + 1 < chain.size(); ++i)
        {
            double radius = INTERIOR_FACTOR * std::max(segmentLength[i - 1], segmentLength[i]);

            // Eligibility (see CORNER_DILUTION_THRESHOLD) compares this
            // edge's own first step against the corner's PRE-clamp,
            // cross-curve-pooled minimum (cornerMinFirstStep) -- whether a
            // finer sibling curve dominated the corner's sizing is a
            // property-1 question, independent of whatever property 2's
            // disjointness clamp separately did to the corner's final
            // radius below.
            if (i == 1 && cornerMinFirstStep.at(chain.front()) >= CORNER_DILUTION_THRESHOLD * segmentLength[i - 1])
            {
                const double cornerRadius = finalRadius.at(chain.front());
                radius = std::max(radius, CORNER_OVERLAP_SLACK * (segmentLength[i - 1] - cornerRadius));
            }
            if (i + 2 == chain.size() &&
                cornerMinFirstStep.at(chain.back()) >= CORNER_DILUTION_THRESHOLD * segmentLength[i])
            {
                const double cornerRadius = finalRadius.at(chain.back());
                radius = std::max(radius, CORNER_OVERLAP_SLACK * (segmentLength[i] - cornerRadius));
            }
            interiorLocalRadius[chain[i]] = radius;
        }
    }

    // Clamp interior points. Brute-force O(featurePoints * points.size())
    // (see nearestUnrelatedDistance()) -- fine at the boundary-discretization
    // point counts RCDT deals with today; revisit with a spatial index if
    // profiling ever shows this dominating (see e.g. the
    // SurfaceTessellation/RCDTRefiner incrementalization history for the
    // project's general "correct first, optimize when it's an actual
    // bottleneck" pattern).
    for (const auto& [interiorPoint, localRadius] : interiorLocalRadius)
        finalRadius[interiorPoint] = std::min(localRadius, DISJOINT_FACTOR * nearestUnrelatedDistance(interiorPoint));

    // A feature point with neither a corner nor an interior local radius
    // (e.g. an isolated corner with no incident edges) gets no protection.
    for (size_t p : orderedFeaturePoints)
        finalRadius.try_emplace(p, 0.0);

    std::unordered_map<size_t, double> weights;
    weights.reserve(finalRadius.size());
    for (const auto& [point, radius] : finalRadius)
        weights[point] = radius * radius;

    // Verify property 1 survived the property-2 clamp; a violation means an
    // unrelated feature passes close enough to this curve, relative to its
    // own local sampling, that no single-pass sizing can satisfy both
    // properties at once -- flagged rather than silently producing a gap in
    // the crease protection (see class comment).
    for (const auto& violation : findUnresolvedSegments(edgeIdToPointIndicesMap, weights, points))
    {
        spdlog::warn("CurveProtectionScheme::computeWeights: protecting balls for edge '{}' do not overlap "
                     "between points {} and {} -- an unrelated feature is too close to this curve for its "
                     "current sampling density",
                     violation.edgeId, violation.nodeId1, violation.nodeId2);
    }

    return weights;
}

std::vector<UnresolvedProtectionSegment> CurveProtectionScheme::findUnresolvedSegments(
    const std::map<std::string, std::vector<size_t>>& edgeIdToPointIndicesMap,
    const std::unordered_map<size_t, double>& weights,
    const std::vector<Point3D>& points)
{
    auto radiusOf = [&weights](size_t point)
    {
        const auto it = weights.find(point);
        return it == weights.end() ? 0.0 : std::sqrt(it->second);
    };

    std::vector<UnresolvedProtectionSegment> unresolved;
    for (const auto& [edgeId, chain] : edgeIdToPointIndicesMap)
    {
        for (size_t k = 0; k + 1 < chain.size(); ++k)
        {
            const double distance = (points[chain[k]] - points[chain[k + 1]]).norm();
            if (radiusOf(chain[k]) + radiusOf(chain[k + 1]) <= distance)
                unresolved.push_back({edgeId, chain[k], chain[k + 1]});
        }
    }
    return unresolved;
}

} // namespace Meshing
