#pragma once

#include "Common/Types.h"

#include <map>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace Meshing
{

/// A curve segment whose two endpoints' final radii don't overlap it -- see
/// CurveProtectionScheme::findUnresolvedSegments().
struct UnresolvedProtectionSegment
{
    std::string edgeId;
    size_t nodeId1 = 0;
    size_t nodeId2 = 0;
};

/**
 * @brief Computes Boissonnat-Oudot-style protecting-ball weights for a
 * discretized curve network (OPE-176).
 *
 * RCDT's initial triangulation inserts every curve/corner sample point as an
 * ordinary (unweighted) Delaunay point today, which is what leaves
 * RestrictedTriangulation::classifyFace() unable to reliably tell which of
 * two candidate surfaces a crease-straddling face belongs to. Inserting
 * these points as WEIGHTED points instead (see RegularPredicates3D), with a
 * radius sized by this class, forces every crease to appear as an exact
 * edge chain in the resulting regular triangulation -- the ambiguity is
 * designed out structurally rather than repaired after the fact.
 *
 * Every computed radius satisfies two properties, WHERE POSSIBLE (see the
 * note on corner-adjacent segments below):
 *
 *  1. Chain connectivity: consecutive balls along the SAME curve overlap
 *     (their radii sum to more than the distance between them), regardless
 *     of how non-uniform the curve's own sampling is -- an angle-based
 *     discretization (see BoundaryDiscretizer3D) can vary segment length
 *     substantially along one curve, so a radius based on the LONGER of a
 *     point's two adjacent segments (never the shorter) is used: for any
 *     segment between two INTERIOR points, both endpoints' radii are
 *     bounded below by a fixed fraction of that segment's own length, which
 *     guarantees overlap unconditionally rather than only for
 *     roughly-uniform sampling. A segment with a corner at one end instead
 *     compensates the interior endpoint against the corner's own (smaller)
 *     radius to force the same overlap in one jump -- see below for when
 *     that compensation is trusted.
 *
 *  2. Disjointness: balls belonging to different, unrelated features (a
 *     different curve, a non-adjacent corner, or a non-consecutive point on
 *     the same curve) never overlap, and no ball ever swallows a point
 *     outside the curve network entirely (e.g. an ordinary face-interior
 *     sample -- see BoundaryDiscretizer3D) -- every point's radius is
 *     clamped to a fraction of its distance to the nearest point it isn't
 *     chain-adjacent to, or to any point not part of the curve network at
 *     all, which (see .cpp) is sufficient to keep every unrelated pair's
 *     balls from summing past their separation.
 *
 * Corners get the smallest ("strong") balls -- sized from the shortest
 * first step of any incident curve, independent of how that curve's own
 * interior points are sized -- so every curve's radii can shrink toward
 * whichever corner is closer without that corner's own ball ever growing to
 * meet them halfway. When a corner is shared with a much more finely-sampled
 * sibling curve, though, that sizing can starve THIS edge's compensation
 * term: bridging the full, uncompensated gap in one jump would inflate the
 * interior point's ball far past this edge's own local scale, encroaching
 * unrelated refinement work well away from the corner (confirmed on
 * RCDTMesherTorusTest's periodic seams, OPE-176). The compensation is
 * therefore only trusted while the corner's radius is still reasonably
 * proportionate to THIS edge's own first step (see
 * CORNER_DILUTION_THRESHOLD in the .cpp); when a much finer sibling has
 * diluted it too far, the segment is left at its natural, uncompensated
 * radius instead, which can leave it unresolved -- see below.
 *
 * Property 1 (including an undercompensated corner-adjacent segment above)
 * and property 2 can both leave a segment unresolved when an unrelated
 * feature, or a differently-scaled corner, is genuinely close to a curve
 * relative to that curve's own sampling density (a small local feature size
 * the discretization didn't anticipate); this class detects that case
 * (findUnresolvedSegments()) and logs it (computeWeights()) rather than
 * silently violating either property. Resolving it means inserting more
 * points into the curve network so radii can close the gap gradually
 * instead of in one jump -- this class doesn't do that itself (it has no
 * geometry access, so it can't place a new point on the true curve) -- see
 * CurveProtectionSubdivider, which drives this class iteratively to do
 * exactly that.
 */
class CurveProtectionScheme
{
public:
    /// edgeIdToPointIndicesMap is the discretized curve network: one entry
    /// per edge, giving its ordered point-index chain (the first and last
    /// entries are corner point indices -- see BoundaryDiscretizer3D).
    /// cornerPointIndices is the set of point indices that are corners.
    /// points gives every point's 3D position, indexed the same way as the
    /// chains. Callers should exclude seam twin edges (see
    /// Topology3D::getSeamCollection()) -- they duplicate an original
    /// edge's points in reverse and would only waste work recomputing the
    /// same radii.
    ///
    /// Returns a weight (protecting radius squared -- see
    /// Node3D::setWeight()) for every point index that appears in
    /// edgeIdToPointIndicesMap or cornerPointIndices. A point absent from
    /// both (an ordinary surface-interior sample) is not a curve/corner
    /// feature; callers should treat it as weight 0.
    static std::unordered_map<size_t, double> computeWeights(
        const std::map<std::string, std::vector<size_t>>& edgeIdToPointIndicesMap,
        const std::unordered_set<size_t>& cornerPointIndices,
        const std::vector<Point3D>& points);

    /// Every consecutive pair in edgeIdToPointIndicesMap whose weights (as
    /// returned by computeWeights(), or any other weight assignment a
    /// caller wants to check) don't overlap -- i.e. every violation of
    /// property 1 (see class comment). weights maps point index to weight
    /// (radius squared); a point absent from weights is treated as radius
    /// 0. Exposed as its own query (mirrors RestrictedTriangulation::
    /// findNonManifoldEdges()) so a caller like CurveProtectionSubdivider
    /// can react to specific violations instead of only seeing
    /// computeWeights()'s log output. computeWeights() itself calls this to
    /// produce that log output, so the two never disagree about what counts
    /// as a violation.
    static std::vector<UnresolvedProtectionSegment> findUnresolvedSegments(
        const std::map<std::string, std::vector<size_t>>& edgeIdToPointIndicesMap,
        const std::unordered_map<size_t, double>& weights,
        const std::vector<Point3D>& points);
};

} // namespace Meshing
