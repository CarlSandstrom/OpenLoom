#pragma once

#include "Common/Types.h"
#include "Meshing/Core/3D/General/ExactArithmetic3D.h"

namespace Meshing
{

/**
 * @brief The orientation determinant of four points, computed adaptively.
 *
 * The determinant is the signed volume of tetrahedron (point0, point1, point2,
 * point3) times six, taken over the rows point0 - point3, point1 - point3 and
 * point2 - point3. RobustPredicates3D reads only its sign; RegularPredicates3D
 * also adds its value into the weighted in-orthosphere determinant.
 *
 * The fast path computes it in plain double alongside the permanent, the sum of
 * magnitudes of its expanded terms. Rounding error scales with the permanent,
 * not with the value, which cancellation can make far smaller: a fast value
 * within SAFETY_FACTOR times its permanent of zero is not trustworthy, and the
 * exact expansion decides. SAFETY_FACTOR is deliberately generous, ~1e8 times
 * the actual worst-case rounding for a determinant this shallow, trading a
 * little fast-path coverage for being unambiguously safe rather than chasing
 * Shewchuk's tightest published bounds.
 */
class OrientationDeterminant3D
{
public:
    static constexpr double SAFETY_FACTOR = 1e-8;

    struct FastResult
    {
        double value = 0.0;
        double permanent = 0.0;
    };

    static FastResult fast(const Point3D& point0, const Point3D& point1, const Point3D& point2, const Point3D& point3);

    static ExactArithmetic3D::Expansion exact(const Point3D& point0,
                                              const Point3D& point1,
                                              const Point3D& point2,
                                              const Point3D& point3);

    /// +1, -1, or 0 (coplanar): the fast value's sign when it clears the bound,
    /// otherwise the exact one.
    static int sign(const Point3D& point0, const Point3D& point1, const Point3D& point2, const Point3D& point3);

    /// The sign of a fast result already computed for these points, falling back
    /// to the exact expansion when it does not clear the bound.
    static int sign(const FastResult& fastResult,
                    const Point3D& point0,
                    const Point3D& point1,
                    const Point3D& point2,
                    const Point3D& point3);
};

} // namespace Meshing
