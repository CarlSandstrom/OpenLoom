#pragma once

#include "Common/Types.h"

namespace Meshing
{

/**
 * @brief Robust (exact) in-orthosphere predicate for weighted points, as a
 * regular triangulation needs.
 *
 * Each point carries a weight (its squared protecting-ball radius), so that a
 * small ball next to a large one is never swallowed by an unweighted
 * neighbour. Only the in-sphere test differs from the unweighted case; the
 * orientation predicate ignores weights, so callers use
 * RobustPredicates3D::orientationSign() directly.
 *
 * A weighted point (p, w) has power distance |x - p|^2 - w to a point x. The
 * in-orthosphere test is the in-sphere determinant with each row's quadratic
 * term x^2+y^2+z^2 reduced by the point's weight: lifting each point to
 * (x, y, z, x^2+y^2+z^2-w) and asking which side of the hyperplane through
 * the four lifted tetrahedron points the lifted query lies. With every weight
 * 0 it is term for term the ordinary circumsphere test (see
 * test_RegularPredicates3D.cpp).
 *
 * The determinant's sign is evaluated directly; no center or radius is
 * formed. Solving for a circumcenter is nearly singular on a near-flat
 * tetrahedron and can be wrong by orders of magnitude (OPE-159).
 *
 * Uses the same exact expansion arithmetic as RobustPredicates3D, through
 * ExactArithmetic3D.
 */
class RegularPredicates3D
{
public:
    /// True if queryPoint (with weight queryWeight) lies inside or exactly on
    /// the orthosphere of the weighted tetrahedron (point0, weight0) ..
    /// (point3, weight3). Ties count as inside, because boundary points often
    /// lie exactly co-orthospherical by construction and must enter the
    /// Bowyer-Watson cavity. Returns false for a degenerate (zero-volume)
    /// tetrahedron, which has no orthosphere whatever the weights.
    static bool insidePointOrthosphere(const Point3D& point0, double weight0,
                                       const Point3D& point1, double weight1,
                                       const Point3D& point2, double weight2,
                                       const Point3D& point3, double weight3,
                                       const Point3D& queryPoint, double queryWeight);
};

} // namespace Meshing
