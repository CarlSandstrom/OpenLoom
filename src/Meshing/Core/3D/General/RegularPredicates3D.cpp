#include "Meshing/Core/3D/General/RegularPredicates3D.h"
#include "Meshing/Core/3D/General/ExactArithmetic3D.h"
#include "Meshing/Core/3D/General/RobustPredicates3D.h"

#include <array>
#include <cmath>

namespace Meshing
{

namespace
{

using Expansion = ExactArithmetic3D::Expansion;

constexpr double SAFETY_FACTOR = 1e-8;

// ---------------------------------------------------------------------------
// The textbook weighted in-sphere ("orthosphere") test is a 5x5 determinant
// with a trailing all-ones column (one row per weighted point: the tet's 4
// vertices plus the query, each [x, y, z, x^2+y^2+z^2-w, 1]). Rather than
// compute that 5x5 directly (which forces every term to use each point's
// ABSOLUTE coordinates, defeating the conditioning RobustPredicates3D's
// query-relative reduction relies on -- confirmed empirically: it roughly
// doubled the exact-arithmetic fallback rate, which is why this doesn't
// build that way), this reduces the 5x5 to the SAME 4x4-relative-to-query
// form RobustPredicates3D::insidePointCircumsphere() uses (small,
// geometrically local magnitudes -- the actual source of that fast path's
// low fallback rate), plus one extra correction term for the query's own
// weight.
//
// Translating all 5 points by -queryPoint doesn't change the determinant
// (adding a linear combination of the x/y/z/"1" columns to the height
// column, to account for the translation, is a column operation and column
// operations don't change a determinant -- this holds independent of the
// weight subtraction, since weights are carried unchanged by translation).
// After translating, the query's own row becomes [0, 0, 0, -queryWeight, 1]
// (mostly zero), so expanding the 5x5 along that row leaves only two
// surviving cofactors:
//
//   det5x5 = queryWeight * orientDet + M4
//
// where:
//  - orientDet is the 4x4 determinant of the tet's 4 points' [x,y,z,1] rows
//    -- translation-invariant, so it's just RobustPredicates3D::
//    orientationSign()'s own underlying quantity (computed here as its own
//    small local helper since that class only exposes the sign, not the
//    value this needs to combine with queryWeight).
//  - M4 is the 4x4 determinant of the tet's 4 points' [dx,dy,dz,dx^2+dy^2+dz^2-w]
//    rows relative to queryPoint -- EXACTLY RobustPredicates3D::
//    insidePointCircumsphere()'s own `determinant` quantity, generalized by
//    subtracting each point's weight from its height term.
//
// Both terms use small, query-relative magnitudes, so this has the same
// fast-path conditioning as the unweighted predicate; when queryWeight == 0
// (refinement's ordinary, unweighted Steiner points -- the overwhelmingly
// common case) the orientDet term is skipped entirely, leaving M4 alone,
// identical in cost to RobustPredicates3D's own insphere test.
// ---------------------------------------------------------------------------

struct FastResult
{
    double value = 0.0;

    // Sum of the absolute values of every term the determinant's cofactor
    // expansion summed to reach `value` -- NOT |value| itself (that's the
    // already-cancelled result, and using it in place of this to bound a
    // *further* computation this value feeds into would understate the true
    // error: |value| can be far smaller than the raw term magnitudes that
    // combined to produce it). This is what a caller combining this result
    // with another quantity must scale its own SAFETY_FACTOR bound from.
    double permanent = 0.0;
};

FastResult fastOrientDet(const Point3D& p0, const Point3D& p1, const Point3D& p2, const Point3D& p3)
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
    return {det, permanent};
}

Expansion exactOrientDet(const Point3D& p0, const Point3D& p1, const Point3D& p2, const Point3D& p3)
{
    Expansion m[3][3];
    const Point3D relativeTo[3] = {p0, p1, p2};
    for (int i = 0; i < 3; ++i)
    {
        m[i][0] = ExactArithmetic3D::expansionFromDifference(relativeTo[i].x(), p3.x());
        m[i][1] = ExactArithmetic3D::expansionFromDifference(relativeTo[i].y(), p3.y());
        m[i][2] = ExactArithmetic3D::expansionFromDifference(relativeTo[i].z(), p3.z());
    }
    return ExactArithmetic3D::det3x3(m);
}

FastResult fastM4(const std::array<Point3D, 4>& p, const std::array<double, 4>& w, const Point3D& queryPoint)
{
    double d[4][4];
    for (int i = 0; i < 4; ++i)
    {
        d[i][0] = p[static_cast<size_t>(i)].x() - queryPoint.x();
        d[i][1] = p[static_cast<size_t>(i)].y() - queryPoint.y();
        d[i][2] = p[static_cast<size_t>(i)].z() - queryPoint.z();
        d[i][3] = d[i][0] * d[i][0] + d[i][1] * d[i][1] + d[i][2] * d[i][2] - w[static_cast<size_t>(i)];
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
    return {det, permanent};
}

Expansion exactM4(const std::array<Point3D, 4>& p, const std::array<double, 4>& w, const Point3D& queryPoint)
{
    Expansion d[4][4];
    for (int i = 0; i < 4; ++i)
    {
        d[i][0] = ExactArithmetic3D::expansionFromDifference(p[static_cast<size_t>(i)].x(), queryPoint.x());
        d[i][1] = ExactArithmetic3D::expansionFromDifference(p[static_cast<size_t>(i)].y(), queryPoint.y());
        d[i][2] = ExactArithmetic3D::expansionFromDifference(p[static_cast<size_t>(i)].z(), queryPoint.z());
        const Expansion squaredNorm = ExactArithmetic3D::squaredNormRelative(d[i][0], d[i][1], d[i][2]);
        d[i][3] = ExactArithmetic3D::expansionSub(
            squaredNorm, ExactArithmetic3D::expansionFromDouble(w[static_cast<size_t>(i)]));
    }

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
        minor[skipRow] = ExactArithmetic3D::det3x3(sub);
    }

    Expansion determinant;
    for (int i = 0; i < 4; ++i)
    {
        Expansion term = ExactArithmetic3D::expansionMul(d[i][3], minor[i]);
        if (i % 2 == 0)
            term = ExactArithmetic3D::expansionNegate(term);
        determinant = ExactArithmetic3D::expansionAdd(determinant, term);
    }
    return determinant;
}

} // namespace

bool RegularPredicates3D::insidePointOrthosphere(const Point3D& p0, double w0,
                                                 const Point3D& p1, double w1,
                                                 const Point3D& p2, double w2,
                                                 const Point3D& p3, double w3,
                                                 const Point3D& queryPoint, double queryWeight)
{
    const int orientation = RobustPredicates3D::orientationSign(p0, p1, p2, p3);
    if (orientation == 0)
        return false;

    const std::array<Point3D, 4> p = {p0, p1, p2, p3};
    const std::array<double, 4> w = {w0, w1, w2, w3};

    int orthosphereSign;
    if (queryWeight == 0.0)
    {
        // det5x5 == M4 exactly when queryWeight is 0 -- skip the orientDet
        // term entirely (see file comment). This is the common case:
        // refinement's ordinary Steiner points are always unweighted.
        const FastResult m4 = fastM4(p, w, queryPoint);
        if (std::abs(m4.value) > SAFETY_FACTOR * m4.permanent)
            orthosphereSign = (m4.value > 0.0) ? 1 : -1;
        else
            orthosphereSign = ExactArithmetic3D::expansionSign(exactM4(p, w, queryPoint));
    }
    else
    {
        const FastResult orient = fastOrientDet(p0, p1, p2, p3);
        const FastResult m4 = fastM4(p, w, queryPoint);
        const double det = queryWeight * orient.value + m4.value;
        // Combine each term's own PERMANENT (not its already-cancelled
        // value -- see FastResult's doc) so the threshold conservatively
        // bounds the true worst-case rounding error of the combined sum,
        // not just of each half in isolation.
        const double bound = SAFETY_FACTOR * (std::abs(queryWeight) * orient.permanent + m4.permanent);

        if (std::abs(det) > bound)
        {
            orthosphereSign = det > 0.0 ? 1 : -1;
        }
        else
        {
            const Expansion exact = ExactArithmetic3D::expansionAdd(
                ExactArithmetic3D::expansionScale(exactOrientDet(p0, p1, p2, p3), queryWeight),
                exactM4(p, w, queryPoint));
            orthosphereSign = ExactArithmetic3D::expansionSign(exact);
        }
    }

    if (orthosphereSign == 0)
        return true;
    return orthosphereSign == orientation;
}

} // namespace Meshing
