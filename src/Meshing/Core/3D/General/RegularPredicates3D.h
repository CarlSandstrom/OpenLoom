#pragma once

#include "Common/Types.h"

namespace Meshing
{

/**
 * @brief Robust (exact) predicate for weighted (regular-triangulation) points.
 *
 * Added for OPE-176's crease protecting balls: Boissonnat-Oudot curve
 * protection needs a *weighted* Delaunay (regular) triangulation, where each
 * inserted point carries a weight (its squared protecting-ball radius) so
 * that a small ball near a large one is never incorrectly swallowed by an
 * ordinary (zero-weight) neighbor. This class provides the one predicate
 * that differs between Delaunay and regular triangulation -- the in-sphere
 * test generalizes to an "in-orthosphere" (power-distance) test; the
 * orientation predicate is unaffected by weights, so callers reuse
 * RobustPredicates3D::orientationSign() directly instead of this class
 * duplicating it.
 *
 * A weighted point (p, w) has power distance pow(x, p) = |x - p|^2 - w to
 * any point x. The regular triangulation's in-orthosphere test is the
 * classic in-sphere determinant with each row's quadratic term
 * (x^2+y^2+z^2) replaced by that term minus the point's own weight --
 * equivalent to lifting each weighted point to the 4D paraboloid point
 * (x, y, z, x^2+y^2+z^2-w) and testing which side of the hyperplane through
 * the tetrahedron's four lifted points the query's lifted point falls on.
 * Setting every weight (including the query's) to 0 makes this identical,
 * term for term, to the unweighted in-sphere determinant, so this predicate
 * reduces exactly to RobustPredicates3D::insidePointCircumsphere() in that
 * case -- confirmed by test (see test_RegularPredicates3D.cpp).
 *
 * Same exact-expansion-arithmetic technique as RobustPredicates3D (see its
 * class comment for why exactness, not just extra precision, matters here),
 * sharing the ExactArithmetic3D toolkit rather than duplicating it.
 */
class RegularPredicates3D
{
public:
    /// True if queryPoint (with weight queryWeight) lies inside or exactly on
    /// the orthogonal sphere of the weighted tetrahedron (p0,w0)..(p3,w3) --
    /// inclusive on ties, same rationale as
    /// RobustPredicates3D::insidePointCircumsphere() (boundary-discretized
    /// points routinely land exactly cospherical/co-orthospherical by
    /// construction). Returns false if (p0,p1,p2,p3) is degenerate
    /// (zero-volume): a degenerate tetrahedron has no valid orthosphere,
    /// regardless of weights.
    static bool insidePointOrthosphere(const Point3D& p0, double w0,
                                       const Point3D& p1, double w1,
                                       const Point3D& p2, double w2,
                                       const Point3D& p3, double w3,
                                       const Point3D& queryPoint, double queryWeight);
};

} // namespace Meshing
