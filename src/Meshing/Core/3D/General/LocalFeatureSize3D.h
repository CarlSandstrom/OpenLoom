#pragma once

#include "Common/Types.h"

#include <string>
#include <vector>

namespace Topology3D
{
class Topology3D;
}

namespace Meshing
{

/// A point sampled on one CAD entity, as consumed by LocalFeatureSize3D.
/// arcLengthAlongCurve is measured from the owning curve's start and is only
/// meaningful when onCurve is true; surface samples leave it at 0.
struct FeatureSample
{
    Point3D position;
    std::string entityId;
    double arcLengthAlongCurve = 0.0;
    bool onCurve = false;
};

/**
 * @brief Local feature size: how far each sampled point is from the nearest
 * *unrelated* part of the boundary (OPE-181).
 *
 * Local feature size is the second thing that forces elements to be small,
 * and it is independent of the first. Curvature says how fast the geometry
 * bends; local feature size says how close other geometry is. A flat plate
 * 0.2 units thick has zero curvature everywhere and still cannot be meshed
 * with 1-unit elements -- triangles would simply bridge across it. Ruppert's
 * refinement guarantee is stated in terms of this quantity for exactly that
 * reason.
 *
 * The codebase already computed a version of this, as a lambda inside
 * CurveProtectionScheme::computeWeights() (nearestUnrelatedDistance), and
 * applied it to one consumer only -- protecting-ball radii, via
 * DISJOINT_FACTOR. Hoisting it here is what lets every consumer of the
 * sizing field see the same answer.
 *
 * ## What counts as "unrelated"
 *
 * Two samples must not constrain each other merely for being neighbours on
 * the same feature, or for sitting either side of a corner they share -- in
 * both cases they are close because the geometry is connected there, not
 * because there is a thin gap to resolve. But a purely topological
 * adjacency test is also wrong, and fails on precisely the case this
 * measure exists for: the two faces of a thin fin ARE adjacent (they share
 * the edge at the fin's tip), yet points in the middle of the fin need
 * small elements. Adjacency alone would exclude them.
 *
 * The discriminator is therefore a single rule: two samples constrain each
 * other when the route between them along the geometry is much longer than
 * the straight line, so that the straight line is a genuine shortcut
 * through space. The route is taken to be
 *
 *  - the curve itself, for two samples on one curve;
 *  - via the nearest edge the two entities share, for samples on two
 *    entities that meet along one;
 *  - nonexistent (so the rule always fires), when they share nothing.
 *
 * Entities meeting at only a CORNER are excluded outright instead, however
 * close they run -- see the .cpp for why local feature size is unbounded
 * below in a wedge and reports sampling density rather than geometry.
 *
 * A junction fails this test and is correctly ignored: two curves meeting
 * at a right angle, sampled at equal distance r from their shared corner,
 * have a route of 2r against a separation of sqrt(2)*r. A thin fin passes
 * it easily: measured at distance L in from the tip, the route is about 2L
 * against a separation of the fin's thickness.
 *
 * One pair is excluded outright rather than routed: a curve that BOUNDS a
 * surface lies on it, so the two are close along their entire shared
 * length by construction and no separation between them can ever describe
 * a thin feature. Routing cannot express that, because the route through
 * their shared edge is short only near that edge while the coincidence
 * runs the whole way along it.
 *
 * ## Known limitation
 *
 * Two samples on the SAME surface never constrain each other, because the
 * geodesic distance between them across that surface is not available here
 * and the straight-line distance cannot stand in for it. A single surface
 * folding to within a small distance of itself is therefore not detected;
 * one that does so via a neighbouring face (the thin fin above) is. Fixing
 * this needs a surface-geodesic estimate, and is deliberately deferred
 * rather than approximated badly -- an over-small size here would propagate
 * through the whole field.
 */
class LocalFeatureSize3D
{
public:
    /// For each sample, the distance to the nearest unrelated sample, or
    /// infinity when the geometry offers no unrelated sample at all (a lone
    /// convex solid whose features are all mutually connected -- a legitimate
    /// "no proximity constraint anywhere" answer, not a failure).
    ///
    /// Cost is quadratic in the sample count. That is deliberate: this runs
    /// once, on a sample set sized by the geometry rather than by the mesh.
    static std::vector<double> compute(const std::vector<FeatureSample>& samples,
                                       const Topology3D::Topology3D& topology);
};

} // namespace Meshing
