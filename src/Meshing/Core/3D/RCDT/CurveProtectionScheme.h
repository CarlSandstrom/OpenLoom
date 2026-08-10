#pragma once

#include "Common/Types.h"

#include <map>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace Meshing
{

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
 * Every computed radius satisfies two properties:
 *
 *  1. Chain connectivity: consecutive balls along the SAME curve overlap
 *     (their radii sum to more than the distance between them), regardless
 *     of how non-uniform the curve's own sampling is -- an angle-based
 *     discretization (see BoundaryDiscretizer3D) can vary segment length
 *     substantially along one curve, so a radius based on the LONGER of a
 *     point's two adjacent segments (never the shorter) is used: for any
 *     segment, both of its endpoints' radii are bounded below by a fixed
 *     fraction of that segment's own length, which guarantees overlap
 *     unconditionally rather than only for roughly-uniform sampling.
 *
 *  2. Disjointness: balls belonging to different, unrelated features (a
 *     different curve, a non-adjacent corner, or a non-consecutive point on
 *     the same curve) never overlap -- every point's radius is clamped to a
 *     fraction of its distance to the nearest point it isn't chain-adjacent
 *     to, which (see .cpp) is sufficient to keep every unrelated pair's
 *     balls from summing past their separation.
 *
 * Corners get the smallest ("strong") balls -- sized from the shortest
 * first step of any incident curve, independent of how that curve's own
 * interior points are sized -- so every curve's radii can shrink toward
 * whichever corner is closer without that corner's own ball ever growing to
 * meet them halfway.
 *
 * Property 1 and property 2 can conflict when an unrelated feature genuinely
 * passes close to a curve relative to that curve's own sampling density (a
 * small local feature size the discretization didn't anticipate); this
 * class detects that case and logs it rather than silently violating either
 * property (see .cpp) -- resolving it requires locally re-discretizing the
 * curve, out of this class's scope.
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
};

} // namespace Meshing
