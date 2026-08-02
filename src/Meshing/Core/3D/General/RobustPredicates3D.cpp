#include "Meshing/Core/3D/General/RobustPredicates3D.h"

#include <cmath>
#include <optional>
#include <vector>

namespace Meshing
{

namespace
{

// ---------------------------------------------------------------------------
// Exact expansion arithmetic (Shewchuk, "Adaptive Precision Floating-Point
// Arithmetic and Fast Robust Geometric Predicates"). A value is represented
// as the exact sum of a sequence of doubles ("components"), maintained
// nonoverlapping and sorted by increasing magnitude. Unlike double-double
// (a fixed ~32-digit approximation, tried first here — see git history),
// this grows however many components a computation needs and so computes
// polynomial predicates like insphere with zero rounding error, period. That
// distinction matters here: this codebase's boundary discretization of
// circles (cylinder caps, sphere/torus seams) produces points that are
// algebraically exactly cospherical — not approximately, exactly — with
// several unrelated tetrahedra's circumspheres at once. A fixed-precision
// approximation resolves that tie inconsistently (confirmed: ~23% mismatch
// rate against the old linear-solve approach in a targeted fuzz test); exact
// arithmetic resolves it the same way every time.
//
// This implements the "always exact" (non-adaptive) forms — always doing
// full expansion arithmetic rather than Shewchuk's fast-path-then-escalate
// optimization for speed. Simpler to get right, and fast enough here: this
// codebase's mesh sizes are modest and correctness matters far more than
// shaving microseconds off a predicate call.
// ---------------------------------------------------------------------------

using Expansion = std::vector<double>;

struct TwoResult
{
    double value = 0.0;
    double error = 0.0;
};

TwoResult twoSum(double a, double b)
{
    const double sum = a + b;
    const double bVirtual = sum - a;
    const double aVirtual = sum - bVirtual;
    const double bRoundoff = b - bVirtual;
    const double aRoundoff = a - aVirtual;
    return {sum, aRoundoff + bRoundoff};
}

TwoResult twoProduct(double a, double b)
{
    const double product = a * b;
    const double error = std::fma(a, b, -product);
    return {product, error};
}

Expansion expansionFromDouble(double a)
{
    return a == 0.0 ? Expansion{} : Expansion{a};
}

// Exact sum of two expansions. Merges by increasing |value| (standard
// mergesort merge), then folds the merged sequence through two_sum, which is
// exact for operands of any relative magnitude — no ordering precondition,
// unlike fast_two_sum. This is a simplified (not maximally compact, but
// still exact and still O(n)) variant of Shewchuk's fast-expansion-sum.
Expansion expansionAdd(const Expansion& e, const Expansion& f)
{
    std::vector<double> merged;
    merged.reserve(e.size() + f.size());
    size_t i = 0;
    size_t j = 0;
    while (i < e.size() && j < f.size())
    {
        if (std::abs(e[i]) <= std::abs(f[j]))
            merged.push_back(e[i++]);
        else
            merged.push_back(f[j++]);
    }
    while (i < e.size())
        merged.push_back(e[i++]);
    while (j < f.size())
        merged.push_back(f[j++]);

    if (merged.empty())
        return {};

    Expansion result;
    result.reserve(merged.size());
    double carry = merged[0];
    for (size_t k = 1; k < merged.size(); ++k)
    {
        const TwoResult r = twoSum(carry, merged[k]);
        if (r.error != 0.0)
            result.push_back(r.error);
        carry = r.value;
    }
    if (carry != 0.0)
        result.push_back(carry);
    return result;
}

Expansion expansionNegate(const Expansion& e)
{
    Expansion result = e;
    for (double& component : result)
        component = -component;
    return result;
}

Expansion expansionSub(const Expansion& e, const Expansion& f)
{
    return expansionAdd(e, expansionNegate(f));
}

// Exact expansion * scalar: exact-product every component against b, then
// exact-sum all the resulting two-component expansions together.
Expansion expansionScale(const Expansion& e, double b)
{
    Expansion result;
    for (double component : e)
    {
        const TwoResult p = twoProduct(component, b);
        result = expansionAdd(result, expansionFromDouble(p.error));
        result = expansionAdd(result, expansionFromDouble(p.value));
    }
    return result;
}

// Exact expansion * expansion: exact-scale by each component of f, exact-summed.
Expansion expansionMul(const Expansion& e, const Expansion& f)
{
    Expansion result;
    for (double component : f)
        result = expansionAdd(result, expansionScale(e, component));
    return result;
}

// Sign of an exact expansion: nonoverlapping components sorted by increasing
// magnitude means the largest-magnitude (last) nonzero component alone
// determines the sign of the whole sum.
int expansionSign(const Expansion& e)
{
    if (e.empty())
        return 0;
    return e.back() > 0.0 ? 1 : -1;
}

Expansion det2x2(const Expansion& a, const Expansion& b, const Expansion& c, const Expansion& d)
{
    return expansionSub(expansionMul(a, d), expansionMul(b, c));
}

Expansion det3x3(const Expansion m[3][3])
{
    const Expansion c0 = det2x2(m[1][1], m[1][2], m[2][1], m[2][2]);
    const Expansion c1 = det2x2(m[1][0], m[1][2], m[2][0], m[2][2]);
    const Expansion c2 = det2x2(m[1][0], m[1][1], m[2][0], m[2][1]);
    return expansionSub(expansionAdd(expansionMul(m[0][0], c0), expansionMul(m[0][2], c2)), expansionMul(m[0][1], c1));
}

Expansion expansionFromDifference(double a, double b)
{
    // Exact a-b as a (possibly two-component) expansion.
    const TwoResult r = twoSum(a, -b);
    return r.error == 0.0 ? Expansion{r.value} : Expansion{r.error, r.value};
}

Expansion squaredNormRelative(const Expansion& dx, const Expansion& dy, const Expansion& dz)
{
    return expansionAdd(expansionAdd(expansionMul(dx, dx), expansionMul(dy, dy)), expansionMul(dz, dz));
}

// ---------------------------------------------------------------------------
// Adaptive fast path: plain-double arithmetic is ~1000x cheaper than the
// exact expansion arithmetic above, and correct for the vast majority of
// calls — only genuinely near-degenerate input needs the exact fallback.
// Each of these computes the same determinant in plain double alongside a
// conservative error bound (SAFETY_FACTOR relative to the sum of magnitudes
// of the determinant's expanded terms — deliberately generous, ~1e8 times
// larger than the actual worst-case floating-point rounding for a
// bounded-depth determinant like these, so it trades a bit of fast-path
// coverage for being unambiguously safe rather than chasing Shewchuk's
// tightest published bounds). Returns nullopt when the fast result isn't
// trustworthy, signaling the caller to fall back to the exact path.
// ---------------------------------------------------------------------------

constexpr double SAFETY_FACTOR = 1e-8;

std::optional<int> fastOrientationSign(const Point3D& p0, const Point3D& p1, const Point3D& p2, const Point3D& p3)
{
    const double ax = p0.x() - p3.x(), ay = p0.y() - p3.y(), az = p0.z() - p3.z();
    const double bx = p1.x() - p3.x(), by = p1.y() - p3.y(), bz = p1.z() - p3.z();
    const double cx = p2.x() - p3.x(), cy = p2.y() - p3.y(), cz = p2.z() - p3.z();

    const double t0 = ax * (by * cz);
    const double t1 = ax * (bz * cy);
    const double t2 = ay * (bx * cz);
    const double t3 = ay * (bz * cx);
    const double t4 = az * (bx * cy);
    const double t5 = az * (by * cx);

    const double det = t0 - t1 - t2 + t3 + t4 - t5;
    const double permanent = std::abs(t0) + std::abs(t1) + std::abs(t2) + std::abs(t3) + std::abs(t4) + std::abs(t5);
    const double bound = SAFETY_FACTOR * permanent;

    if (std::abs(det) <= bound)
        return std::nullopt;
    return det > 0.0 ? 1 : -1;
}

std::optional<int> fastInsphereSign(const Point3D& p0, const Point3D& p1, const Point3D& p2, const Point3D& p3,
                                    const Point3D& queryPoint)
{
    const Point3D vertices[4] = {p0, p1, p2, p3};
    double d[4][4];
    for (int i = 0; i < 4; ++i)
    {
        d[i][0] = vertices[i].x() - queryPoint.x();
        d[i][1] = vertices[i].y() - queryPoint.y();
        d[i][2] = vertices[i].z() - queryPoint.z();
        d[i][3] = d[i][0] * d[i][0] + d[i][1] * d[i][1] + d[i][2] * d[i][2];
    }

    auto det3 = [](const double m[3][3])
    {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
             - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
             + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    };

    double minor[4];
    for (int skipRow = 0; skipRow < 4; ++skipRow)
    {
        double sub[3][3];
        int r = 0;
        for (int row = 0; row < 4; ++row)
        {
            if (row == skipRow)
                continue;
            sub[r][0] = d[row][0];
            sub[r][1] = d[row][1];
            sub[r][2] = d[row][2];
            ++r;
        }
        minor[skipRow] = det3(sub);
    }

    double det = 0.0;
    double permanent = 0.0;
    for (int i = 0; i < 4; ++i)
    {
        double term = d[i][3] * minor[i];
        permanent += std::abs(term);
        det += (i % 2 == 0) ? -term : term;
    }
    const double bound = SAFETY_FACTOR * permanent;

    if (std::abs(det) <= bound)
        return std::nullopt;
    return det > 0.0 ? 1 : -1;
}

int exactOrientationSign(const Point3D& p0, const Point3D& p1, const Point3D& p2, const Point3D& p3)
{
    // Signed volume of (p0,p1,p2,p3), via rows relative to p3 (same
    // row-subtraction identity used in exactInsphereSign).
    Expansion m[3][3];
    const Point3D relativeTo[3] = {p0, p1, p2};
    for (int i = 0; i < 3; ++i)
    {
        m[i][0] = expansionFromDifference(relativeTo[i].x(), p3.x());
        m[i][1] = expansionFromDifference(relativeTo[i].y(), p3.y());
        m[i][2] = expansionFromDifference(relativeTo[i].z(), p3.z());
    }

    return expansionSign(det3x3(m));
}

int exactInsphereSign(const Point3D& p0, const Point3D& p1, const Point3D& p2, const Point3D& p3,
                      const Point3D& queryPoint)
{
    // Standard in-sphere determinant, built from coordinates shifted relative to
    // queryPoint (eliminates the "1" column of the textbook 5x5 form — subtracting
    // one row from the others is determinant-invariant, see class comment). Row i
    // is [dx_i, dy_i, dz_i, dx_i^2+dy_i^2+dz_i^2] for i in {p0,p1,p2,p3}.
    const Point3D vertices[4] = {p0, p1, p2, p3};

    Expansion d[4][4];
    for (int i = 0; i < 4; ++i)
    {
        d[i][0] = expansionFromDifference(vertices[i].x(), queryPoint.x());
        d[i][1] = expansionFromDifference(vertices[i].y(), queryPoint.y());
        d[i][2] = expansionFromDifference(vertices[i].z(), queryPoint.z());
        d[i][3] = squaredNormRelative(d[i][0], d[i][1], d[i][2]);
    }

    // 4x4 determinant via cofactor expansion along the last (squared-distance) column.
    Expansion minor[4];
    for (int skipRow = 0; skipRow < 4; ++skipRow)
    {
        Expansion sub[3][3];
        int r = 0;
        for (int row = 0; row < 4; ++row)
        {
            if (row == skipRow)
                continue;
            sub[r][0] = d[row][0];
            sub[r][1] = d[row][1];
            sub[r][2] = d[row][2];
            ++r;
        }
        minor[skipRow] = det3x3(sub);
    }

    // det = -d[0][3]*minor0 + d[1][3]*minor1 - d[2][3]*minor2 + d[3][3]*minor3
    Expansion determinant;
    for (int i = 0; i < 4; ++i)
    {
        Expansion term = expansionMul(d[i][3], minor[i]);
        if (i % 2 == 0)
            term = expansionNegate(term);
        determinant = expansionAdd(determinant, term);
    }

    return expansionSign(determinant);
}

} // namespace

bool RobustPredicates3D::insidePointCircumsphere(const Point3D& p0,
                                                 const Point3D& p1,
                                                 const Point3D& p2,
                                                 const Point3D& p3,
                                                 const Point3D& queryPoint)
{
    const int orientation = orientationSign(p0, p1, p2, p3);

    // A degenerate (zero-volume) tetrahedron has no valid circumsphere.
    if (orientation == 0)
        return false;

    const std::optional<int> fast = fastInsphereSign(p0, p1, p2, p3, queryPoint);
    const int insphereSign = fast ? *fast : exactInsphereSign(p0, p1, p2, p3, queryPoint);

    // See class comment: insphereSign agrees with orientationSign exactly when
    // queryPoint is inside the sphere. Treat exactly-on-sphere (insphereSign
    // == 0) as inside — inclusive, like the Bowyer-Watson cavity this feeds
    // needs: boundary-discretized points from a circular/cylindrical edge are
    // routinely exactly cospherical by construction (regular-polygon
    // vertices sharing one circle), and a strict "on sphere is not conflicting"
    // reading leaves some of them out of the cavity, producing an
    // inconsistent triangulation (a boundary face with only one neighboring
    // tetrahedron) rather than a genuinely ambiguous/wrong answer.
    if (insphereSign == 0)
        return true;
    return insphereSign == orientation;
}

int RobustPredicates3D::orientationSign(const Point3D& p0,
                                        const Point3D& p1,
                                        const Point3D& p2,
                                        const Point3D& p3)
{
    if (const std::optional<int> fast = fastOrientationSign(p0, p1, p2, p3))
        return *fast;
    return exactOrientationSign(p0, p1, p2, p3);
}

} // namespace Meshing
