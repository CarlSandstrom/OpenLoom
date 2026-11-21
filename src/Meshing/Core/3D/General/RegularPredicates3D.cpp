#include "Meshing/Core/3D/General/RegularPredicates3D.h"
#include "Meshing/Core/3D/General/ExactArithmetic3D.h"
#include "Meshing/Core/3D/General/OrientationDeterminant3D.h"

#include <array>
#include <cmath>

namespace Meshing
{

namespace
{

using Expansion = ExactArithmetic3D::Expansion;

// A fast value with the permanent its rounding error scales with; a caller
// combining it with another quantity scales the bound from both permanents.
using FastResult = OrientationDeterminant3D::FastResult;

constexpr double SAFETY_FACTOR = OrientationDeterminant3D::SAFETY_FACTOR;

// The textbook weighted in-sphere ("orthosphere") test is a 5x5 determinant
// with one row [x, y, z, x^2+y^2+z^2-w, 1] per weighted point: the
// tetrahedron's four vertices plus the query. Computing it directly uses
// absolute coordinates, which measurably doubled the exact-arithmetic
// fallback rate. Instead every point is translated by -queryPoint, which
// leaves the determinant unchanged (a column operation), and the query's row
// becomes [0, 0, 0, -queryWeight, 1]. Expanding along that row leaves two
// cofactors:
//
//   det5x5 = queryWeight * orientationDeterminant + M4
//
// where:
//  - orientationDeterminant is the 4x4 determinant of the tetrahedron's
//    [x, y, z, 1] rows: OrientationDeterminant3D, whose sign is also the
//    tetrahedron's orientation, so it is computed once for both.
//  - M4 is the 4x4 determinant of the tetrahedron's
//    [dx, dy, dz, dx^2+dy^2+dz^2-w] rows relative to queryPoint: the
//    unweighted in-sphere determinant with each point's weight subtracted
//    from its height term.
//
// Both terms use small, query-relative magnitudes, so the fast path is as
// well conditioned as the unweighted predicate. When queryWeight == 0 (every
// ordinary refinement point) the orientation term is skipped, leaving the
// cost of an unweighted in-sphere test.

FastResult fastM4(const std::array<Point3D, 4>& points, const std::array<double, 4>& weights,
                  const Point3D& queryPoint);
Expansion exactM4(const std::array<Point3D, 4>& points, const std::array<double, 4>& weights,
                  const Point3D& queryPoint);

} // namespace

bool RegularPredicates3D::insidePointOrthosphere(const Point3D& point0, double weight0,
                                                 const Point3D& point1, double weight1,
                                                 const Point3D& point2, double weight2,
                                                 const Point3D& point3, double weight3,
                                                 const Point3D& queryPoint, double queryWeight)
{
    auto orientationDeterminant = OrientationDeterminant3D::fast(point0, point1, point2, point3);
    int orientation = OrientationDeterminant3D::sign(orientationDeterminant, point0, point1, point2, point3);
    if (orientation == 0)
        return false;

    std::array<Point3D, 4> points = {point0, point1, point2, point3};
    std::array<double, 4> weights = {weight0, weight1, weight2, weight3};

    int orthosphereSign;
    if (queryWeight == 0.0)
    {
        // det5x5 == M4 exactly when queryWeight is 0 (see the file comment).
        FastResult m4 = fastM4(points, weights, queryPoint);
        if (std::abs(m4.value) > SAFETY_FACTOR * m4.permanent)
            orthosphereSign = (m4.value > 0.0) ? 1 : -1;
        else
            orthosphereSign = ExactArithmetic3D::expansionSign(exactM4(points, weights, queryPoint));
    }
    else
    {
        FastResult m4 = fastM4(points, weights, queryPoint);
        double determinant = queryWeight * orientationDeterminant.value + m4.value;
        // Bound from each term's permanent, not its cancelled value, so the
        // threshold covers the rounding error of the combined sum.
        double bound = SAFETY_FACTOR * (std::abs(queryWeight) * orientationDeterminant.permanent + m4.permanent);

        if (std::abs(determinant) > bound)
        {
            orthosphereSign = determinant > 0.0 ? 1 : -1;
        }
        else
        {
            Expansion exact = ExactArithmetic3D::expansionAdd(
                ExactArithmetic3D::expansionScale(OrientationDeterminant3D::exact(point0, point1, point2, point3), queryWeight),
                exactM4(points, weights, queryPoint));
            orthosphereSign = ExactArithmetic3D::expansionSign(exact);
        }
    }

    return orthosphereSign == 0 || orthosphereSign == orientation;
}

namespace
{

double fastDeterminant3x3(const double matrix[3][3])
{
    return matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) -
           matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]) +
           matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
}

// minors[skipRow] is the 3x3 determinant of the first three columns of rows,
// with row skipRow removed. Shared by the double and the Expansion path.
template <typename Element, typename Determinant3x3>
void computeMinors(const Element (&rows)[4][4], Element (&minors)[4], Determinant3x3 determinant3x3)
{
    for (size_t skipRow = 0; skipRow < 4; ++skipRow)
    {
        Element subMatrix[3][3];
        size_t subRow = 0;
        for (size_t row = 0; row < 4; ++row)
        {
            if (row == skipRow) continue;
            subMatrix[subRow][0] = rows[row][0];
            subMatrix[subRow][1] = rows[row][1];
            subMatrix[subRow][2] = rows[row][2];
            ++subRow;
        }
        minors[skipRow] = determinant3x3(subMatrix);
    }
}

FastResult fastM4(const std::array<Point3D, 4>& points, const std::array<double, 4>& weights, const Point3D& queryPoint)
{
    double rows[4][4];
    for (size_t i = 0; i < 4; ++i)
    {
        rows[i][0] = points[i].x() - queryPoint.x();
        rows[i][1] = points[i].y() - queryPoint.y();
        rows[i][2] = points[i].z() - queryPoint.z();
        rows[i][3] = rows[i][0] * rows[i][0] + rows[i][1] * rows[i][1] + rows[i][2] * rows[i][2] - weights[i];
    }

    double minor[4];
    computeMinors(rows, minor, fastDeterminant3x3);

    double determinant = 0.0;
    double permanent = 0.0;
    for (size_t i = 0; i < 4; ++i)
    {
        double term = rows[i][3] * minor[i];
        permanent += std::abs(term);
        determinant += (i % 2 == 0) ? -term : term;
    }
    return {determinant, permanent};
}

Expansion exactM4(const std::array<Point3D, 4>& points, const std::array<double, 4>& weights, const Point3D& queryPoint)
{
    Expansion rows[4][4];
    for (size_t i = 0; i < 4; ++i)
    {
        rows[i][0] = ExactArithmetic3D::expansionFromDifference(points[i].x(), queryPoint.x());
        rows[i][1] = ExactArithmetic3D::expansionFromDifference(points[i].y(), queryPoint.y());
        rows[i][2] = ExactArithmetic3D::expansionFromDifference(points[i].z(), queryPoint.z());
        Expansion squaredNorm = ExactArithmetic3D::squaredNormRelative(rows[i][0], rows[i][1], rows[i][2]);
        rows[i][3] = ExactArithmetic3D::expansionSub(squaredNorm, ExactArithmetic3D::expansionFromDouble(weights[i]));
    }

    Expansion minor[4];
    computeMinors(rows, minor, ExactArithmetic3D::det3x3);

    Expansion determinant;
    for (size_t i = 0; i < 4; ++i)
    {
        Expansion term = ExactArithmetic3D::expansionMul(rows[i][3], minor[i]);
        if (i % 2 == 0) term = ExactArithmetic3D::expansionNegate(term);
        determinant = ExactArithmetic3D::expansionAdd(determinant, term);
    }
    return determinant;
}

} // namespace

} // namespace Meshing
