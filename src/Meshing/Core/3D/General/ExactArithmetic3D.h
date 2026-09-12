#pragma once

#include <vector>

namespace Meshing
{

/**
 * @brief Exact expansion arithmetic (Shewchuk, "Adaptive Precision
 * Floating-Point Arithmetic and Fast Robust Geometric Predicates").
 *
 * A value is represented as the exact sum of a sequence of doubles
 * ("components"), maintained nonoverlapping and sorted by increasing
 * magnitude -- see RobustPredicates3D.h for why exactness (not merely extra
 * precision) matters for this codebase's predicates.
 *
 * Extracted from RobustPredicates3D so RegularPredicates3D (OPE-176's
 * weighted/regular-triangulation predicates) can reuse the same toolkit
 * instead of duplicating it: both classes need identical exact-sum,
 * exact-product, and determinant building blocks, only the geometric
 * predicate built on top differs.
 *
 * This implements the "always exact" (non-adaptive) forms only -- always
 * doing full expansion arithmetic rather than escalating from a fast
 * plain-double path. Callers needing the fast path build their own
 * alongside a conservative error bound and fall back to these only when
 * that bound isn't comfortably cleared (see RobustPredicates3D.cpp and
 * RegularPredicates3D.cpp).
 */
class ExactArithmetic3D
{
public:
    using Expansion = std::vector<double>;

    struct TwoResult
    {
        double value = 0.0;
        double error = 0.0;
    };

    static TwoResult twoSum(double a, double b);
    static TwoResult twoProduct(double a, double b);

    static Expansion expansionFromDouble(double a);

    /// Exact a-b as a (possibly two-component) expansion.
    static Expansion expansionFromDifference(double a, double b);

    /// Exact sum of two expansions -- a simplified (not maximally compact,
    /// but still exact and still O(n)) variant of Shewchuk's
    /// fast-expansion-sum.
    static Expansion expansionAdd(const Expansion& e, const Expansion& f);

    static Expansion expansionNegate(const Expansion& e);
    static Expansion expansionSub(const Expansion& e, const Expansion& f);

    /// Exact expansion * scalar.
    static Expansion expansionScale(const Expansion& e, double b);

    /// Exact expansion * expansion.
    static Expansion expansionMul(const Expansion& e, const Expansion& f);

    /// Sign of an exact expansion: nonoverlapping components sorted by
    /// increasing magnitude means the largest-magnitude (last) nonzero
    /// component alone determines the sign of the whole sum.
    static int expansionSign(const Expansion& e);

    static Expansion det2x2(const Expansion& a, const Expansion& b, const Expansion& c, const Expansion& d);
    static Expansion det3x3(const Expansion m[3][3]);

    /// Cofactor expansion along the first column.
    static Expansion det4x4(const Expansion m[4][4]);

    static Expansion squaredNormRelative(const Expansion& dx, const Expansion& dy, const Expansion& dz);
};

} // namespace Meshing
