#pragma once

#include "Common/Types.h"

namespace Meshing
{

/**
 * @brief Robust (exact) geometric predicates.
 *
 * Each predicate computes the exact sign of a quantity that plain IEEE double
 * arithmetic cannot reliably distinguish from zero near a degenerate
 * configuration, rather than comparing against an epsilon. Every intermediate
 * sum and product uses Shewchuk-style exact expansion arithmetic
 * (ExactArithmetic3D). RegularPredicates3D's in-orthosphere test is built the
 * same way.
 *
 * Exact, not merely higher-precision, matters here: the boundary
 * discretization of circles (cylinder caps, sphere/torus seams) produces
 * points that are algebraically exactly cospherical. Double-double
 * (~32 decimal digits) precision was tried and resolved those ties
 * inconsistently (~23% disagreement in a targeted fuzz test of the in-sphere
 * test), because no fixed-width approximation is guaranteed to compute zero
 * when the true answer is zero. Exact expansion arithmetic does, every time.
 *
 * Exact expansion arithmetic is ~1000x more expensive than plain double, so
 * the orientation test first tries a plain-double fast path with a
 * conservative error bound (see .cpp) and falls back to the exact path only
 * when the fast result is not comfortably outside that bound: Shewchuk's
 * "adaptive precision" strategy.
 */
class RobustPredicates3D
{
public:
    /// Sign of the signed volume of tetrahedron (point0,point1,point2,point3):
    /// +1, -1, or 0 (degenerate/coplanar).
    static int orientationSign(const Point3D& point0,
                               const Point3D& point1,
                               const Point3D& point2,
                               const Point3D& point3);

    /// True if segment (segmentStart,segmentEnd) crosses the interior of
    /// triangle (vertex0,vertex1,vertex2), exactly, built entirely from
    /// orientationSign(): the segment's ends must be on opposite sides of the
    /// triangle's plane, and the orientation of the segment against each of
    /// the triangle's 3 edges must agree in sign. The plane test is half-open:
    /// an end lying exactly in the plane counts as on its positive side, so a
    /// segment ending there crosses only when its other end is on the
    /// negative side. A segment lying in the plane, or passing through an
    /// edge or a vertex (an edge orientation test landing exactly on 0), is
    /// not a crossing: returns false.
    ///
    /// Reducing "does this segment cross surface S" to "does it cross any
    /// triangle in a fixed tessellation of S" (SurfaceTessellation) turns a
    /// floating-point-fragile question (near-tangent distance-to-surface
    /// comparisons) into this purely point-based predicate.
    static bool segmentCrossesTriangle(const Point3D& segmentStart,
                                       const Point3D& segmentEnd,
                                       const Point3D& vertex0,
                                       const Point3D& vertex1,
                                       const Point3D& vertex2);
};

} // namespace Meshing
