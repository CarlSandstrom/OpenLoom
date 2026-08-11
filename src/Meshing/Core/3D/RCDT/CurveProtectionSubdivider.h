#pragma once

#include <unordered_map>

namespace Geometry3D
{
class GeometryCollection3D;
} // namespace Geometry3D

namespace Topology3D
{
class Topology3D;
} // namespace Topology3D

namespace Meshing
{

struct DiscretizationResult3D;

/**
 * @brief Inserts extra curve points where CurveProtectionScheme's sizing
 * alone can't satisfy both its properties, then returns the resulting
 * weights (OPE-176).
 *
 * CurveProtectionScheme sizes the EXISTING sample points on a curve; it has
 * no way to fix a genuine local-feature-size conflict, where a corner's
 * disjointness-forced radius (property 2) and its neighbor's natural radius
 * are simply too far apart for two points to bridge (property 1) --
 * confirmed on RCDTMesherTorusTest's periodic seam. The standard
 * Boissonnat-Oudot answer is to insert more points into the gap, each
 * radius growing until it reaches the curve's normal scale, rather than
 * trying to resize the two existing points to do an impossible job.
 *
 * This class drives that process: repeatedly call CurveProtectionScheme::
 * computeWeights() and findUnresolvedSegments(), split the first violation
 * at its arc-length midpoint (via the edge's own geometry, so the new point
 * lies exactly on the true curve -- the same technique
 * CurveSegmentOperations::computeSplitPoint() uses for segment splits during
 * refinement), and repeat -- until nothing is left unresolved, or the next
 * split would produce a segment shorter than minimumEdgeLength. That floor
 * is the same "leave it, don't retry forever" termination guarantee
 * RCDTRefiner's unrefinable sets already rely on elsewhere: a genuinely
 * unfixable defect (this scheme's assumptions about local feature size
 * still don't hold even at the mesh's finest useful scale) is left as a
 * logged, permanent gap rather than chased indefinitely.
 *
 * Mutates discretizationResult in place (appends new points and splices
 * them into the affected edges' chains), unlike CurveProtectionScheme
 * itself, which only ever reads its input -- this is the class that needed
 * geometry access to place a new point on the true curve, which is exactly
 * why it's kept separate from CurveProtectionScheme's pure position/weight
 * math (see that class's own doc comment).
 */
class CurveProtectionSubdivider
{
public:
    /// geometry resolves each edgeId to its true curve (for placing new
    /// points exactly on it) and to check IEdge3D::isDegenerate(); topology
    /// resolves seam twins (SeamCollection::isSeamTwin()) -- both edge
    /// categories are excluded from consideration, same as
    /// CurveProtectionScheme's caller already excludes them from weight
    /// computation (a seam twin duplicates its original edge's points in
    /// reverse; a degenerate edge, e.g. a sphere's polar edge, has no real
    /// length to subdivide). minimumEdgeLength is the same size floor
    /// RCDTRefiner resolves at the start of refinement.
    static std::unordered_map<size_t, double> subdivide(
        DiscretizationResult3D& discretizationResult,
        const Topology3D::Topology3D& topology,
        const Geometry3D::GeometryCollection3D& geometry,
        double minimumEdgeLength);
};

} // namespace Meshing
