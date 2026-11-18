#include "Meshing/Core/3D/General/RobustPredicates3D.h"
#include "Meshing/Core/3D/General/ExactArithmetic3D.h"

#include <cmath>
#include <optional>

namespace Meshing
{

namespace
{

// The fast path computes the determinant in plain double alongside a
// conservative error bound: SAFETY_FACTOR times the sum of magnitudes of the
// determinant's expanded terms. That is deliberately generous, ~1e8 times the
// actual worst-case rounding for a determinant this shallow, trading a little
// fast-path coverage for being unambiguously safe rather than chasing
// Shewchuk's tightest published bounds. It returns nullopt when the result is
// not trustworthy, and the caller falls back to the exact path.

constexpr double SAFETY_FACTOR = 1e-8;

std::optional<int> fastOrientationSign(const Point3D& point0,
                                       const Point3D& point1,
                                       const Point3D& point2,
                                       const Point3D& point3);
int exactOrientationSign(const Point3D& point0, const Point3D& point1, const Point3D& point2, const Point3D& point3);
int halfOpenPlaneSide(const Point3D& vertex0, const Point3D& vertex1, const Point3D& vertex2, const Point3D& point);

} // namespace

int RobustPredicates3D::orientationSign(const Point3D& point0,
                                        const Point3D& point1,
                                        const Point3D& point2,
                                        const Point3D& point3)
{
    if (auto fastSign = fastOrientationSign(point0, point1, point2, point3))
        return *fastSign;
    return exactOrientationSign(point0, point1, point2, point3);
}

bool RobustPredicates3D::segmentCrossesTriangle(const Point3D& segmentStart,
                                                const Point3D& segmentEnd,
                                                const Point3D& vertex0,
                                                const Point3D& vertex1,
                                                const Point3D& vertex2)
{
    // The segment's ends must be on opposite sides of the triangle's plane,
    // an end lying in it counting as on its positive side. Equal sides also
    // reject the coplanar case.
    int startSide = halfOpenPlaneSide(vertex0, vertex1, vertex2, segmentStart);
    int endSide = halfOpenPlaneSide(vertex0, vertex1, vertex2, segmentEnd);
    if (startSide == endSide)
        return false;

    // The plane crossing point (never computed explicitly) is inside the
    // triangle iff the segment passes the same side of all 3 edges, taken in
    // a consistent winding order around the triangle -- i.e. these three
    // orientation tests all agree in sign. Any of them landing on exactly 0
    // means the crossing touches an edge or vertex rather than the
    // triangle's interior, so it's rejected rather than treated as a match.
    int edge01 = orientationSign(segmentStart, segmentEnd, vertex0, vertex1);
    int edge12 = orientationSign(segmentStart, segmentEnd, vertex1, vertex2);
    int edge20 = orientationSign(segmentStart, segmentEnd, vertex2, vertex0);
    return edge01 != 0 && edge01 == edge12 && edge12 == edge20;
}

namespace
{

std::optional<int> fastOrientationSign(const Point3D& point0,
                                       const Point3D& point1,
                                       const Point3D& point2,
                                       const Point3D& point3)
{
    double ax = point0.x() - point3.x();
    double ay = point0.y() - point3.y();
    double az = point0.z() - point3.z();
    double bx = point1.x() - point3.x();
    double by = point1.y() - point3.y();
    double bz = point1.z() - point3.z();
    double cx = point2.x() - point3.x();
    double cy = point2.y() - point3.y();
    double cz = point2.z() - point3.z();

    double term0 = ax * (by * cz);
    double term1 = ax * (bz * cy);
    double term2 = ay * (bx * cz);
    double term3 = ay * (bz * cx);
    double term4 = az * (bx * cy);
    double term5 = az * (by * cx);

    double determinant = term0 - term1 - term2 + term3 + term4 - term5;
    double permanent = std::abs(term0) + std::abs(term1) + std::abs(term2) + std::abs(term3) + std::abs(term4) +
                       std::abs(term5);
    double bound = SAFETY_FACTOR * permanent;

    if (std::abs(determinant) <= bound)
        return std::nullopt;
    return determinant > 0.0 ? 1 : -1;
}

int exactOrientationSign(const Point3D& point0, const Point3D& point1, const Point3D& point2, const Point3D& point3)
{
    // Signed volume of (point0,point1,point2,point3), via rows relative to point3.
    ExactArithmetic3D::Expansion matrix[3][3];
    Point3D rowPoints[3] = {point0, point1, point2};
    for (int i = 0; i < 3; ++i)
    {
        matrix[i][0] = ExactArithmetic3D::expansionFromDifference(rowPoints[i].x(), point3.x());
        matrix[i][1] = ExactArithmetic3D::expansionFromDifference(rowPoints[i].y(), point3.y());
        matrix[i][2] = ExactArithmetic3D::expansionFromDifference(rowPoints[i].z(), point3.z());
    }

    return ExactArithmetic3D::expansionSign(ExactArithmetic3D::det3x3(matrix));
}

// The side of the triangle's plane the point lies on, a point exactly in the plane counting as
// on the positive side. A dual edge can end at an orthocenter that lies on a planar face to
// within rounding -- the exact one may be closer than one ulp -- and every face around it then
// has a dual edge ending there. Assigning the end one side decides all of them consistently, so
// exactly the faces whose other end lies opposite are crossed (OPE-214).
int halfOpenPlaneSide(const Point3D& vertex0, const Point3D& vertex1, const Point3D& vertex2, const Point3D& point)
{
    return RobustPredicates3D::orientationSign(vertex0, vertex1, vertex2, point) < 0 ? -1 : 1;
}

} // namespace

} // namespace Meshing
