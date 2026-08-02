#pragma once

#include "Common/Types.h"

namespace Meshing
{

/**
 * @brief Robust (exact) geometric predicates for 3D Delaunay operations.
 *
 * ElementGeometry3D::computeCircumscribingSphere() solves a 3x3 linear system for the
 * circumcenter, then callers compare distances against the resulting radius. For a
 * near-degenerate (nearly coplanar/flat) tetrahedron that system is nearly singular:
 * the solved center can be wrong by orders of magnitude — radii in the thousands for
 * a mesh a few units across are not unusual — which corrupts every conflict test that
 * relies on it (see OPE-159).
 *
 * insidePointCircumsphere() avoids that instability entirely: it never forms an
 * explicit center or radius. It evaluates the sign of the standard 4x4 in-sphere
 * determinant directly, using Shewchuk-style exact expansion arithmetic (see .cpp)
 * for every intermediate sum and product instead of plain IEEE double.
 *
 * Exact, not merely higher-precision, matters here: this codebase's boundary
 * discretization of circles (cylinder caps, sphere/torus seams) produces points
 * that are algebraically exactly cospherical with each other's circumspheres —
 * not approximately, exactly, regardless of how many extra digits a fixed-width
 * approximation carries. A first attempt at this class used double-double
 * (~32 decimal digits) precision instead of true exact arithmetic; it resolved
 * those exact ties inconsistently (~23% disagreement with the old approach in a
 * targeted fuzz test) because the true mathematical answer is zero, and no
 * fixed-width approximation is guaranteed to compute zero for a case that isn't
 * actually zero-width in its own representation. Exact expansion arithmetic
 * computes precisely zero for those cases, every time.
 *
 * Exact expansion arithmetic is ~1000x more expensive than plain double
 * arithmetic, so both predicates first try a plain-double fast path with a
 * conservative error bound (see .cpp) and only fall back to the exact path
 * when the fast result isn't comfortably outside that bound — Shewchuk's
 * "adaptive precision" strategy, so the common (well-conditioned) case stays
 * cheap and only genuinely near-degenerate input pays for exactness.
 */
class RobustPredicates3D
{
public:
    /// True if queryPoint lies inside or exactly on the circumsphere of tetrahedron
    /// (p0,p1,p2,p3) — inclusive, matching what Bowyer-Watson needs (see .cpp).
    /// Correct regardless of the tetrahedron's vertex winding/orientation.
    static bool insidePointCircumsphere(const Point3D& p0,
                                        const Point3D& p1,
                                        const Point3D& p2,
                                        const Point3D& p3,
                                        const Point3D& queryPoint);

    /// Sign of the signed volume of tetrahedron (p0,p1,p2,p3): +1, -1, or 0 (degenerate/coplanar).
    static int orientationSign(const Point3D& p0,
                               const Point3D& p1,
                               const Point3D& p2,
                               const Point3D& p3);
};

} // namespace Meshing
