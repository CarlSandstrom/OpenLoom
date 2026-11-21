#include "Meshing/Core/3D/General/OrientationDeterminant3D.h"

#include <cmath>

namespace Meshing
{

OrientationDeterminant3D::FastResult OrientationDeterminant3D::fast(const Point3D& point0,
                                                                    const Point3D& point1,
                                                                    const Point3D& point2,
                                                                    const Point3D& point3)
{
    double edge0X = point0.x() - point3.x();
    double edge0Y = point0.y() - point3.y();
    double edge0Z = point0.z() - point3.z();
    double edge1X = point1.x() - point3.x();
    double edge1Y = point1.y() - point3.y();
    double edge1Z = point1.z() - point3.z();
    double edge2X = point2.x() - point3.x();
    double edge2Y = point2.y() - point3.y();
    double edge2Z = point2.z() - point3.z();

    double term0 = edge0X * (edge1Y * edge2Z);
    double term1 = edge0X * (edge1Z * edge2Y);
    double term2 = edge0Y * (edge1X * edge2Z);
    double term3 = edge0Y * (edge1Z * edge2X);
    double term4 = edge0Z * (edge1X * edge2Y);
    double term5 = edge0Z * (edge1Y * edge2X);

    double determinant = term0 - term1 - term2 + term3 + term4 - term5;
    double permanent = std::abs(term0) + std::abs(term1) + std::abs(term2) + std::abs(term3) + std::abs(term4) +
                       std::abs(term5);
    return {determinant, permanent};
}

ExactArithmetic3D::Expansion OrientationDeterminant3D::exact(const Point3D& point0,
                                                             const Point3D& point1,
                                                             const Point3D& point2,
                                                             const Point3D& point3)
{
    ExactArithmetic3D::Expansion matrix[3][3];
    Point3D rowPoints[3] = {point0, point1, point2};
    for (int i = 0; i < 3; ++i)
    {
        matrix[i][0] = ExactArithmetic3D::expansionFromDifference(rowPoints[i].x(), point3.x());
        matrix[i][1] = ExactArithmetic3D::expansionFromDifference(rowPoints[i].y(), point3.y());
        matrix[i][2] = ExactArithmetic3D::expansionFromDifference(rowPoints[i].z(), point3.z());
    }
    return ExactArithmetic3D::det3x3(matrix);
}

int OrientationDeterminant3D::sign(const Point3D& point0, const Point3D& point1, const Point3D& point2, const Point3D& point3)
{
    return sign(fast(point0, point1, point2, point3), point0, point1, point2, point3);
}

int OrientationDeterminant3D::sign(const FastResult& fastResult,
                                   const Point3D& point0,
                                   const Point3D& point1,
                                   const Point3D& point2,
                                   const Point3D& point3)
{
    if (std::abs(fastResult.value) > SAFETY_FACTOR * fastResult.permanent) return fastResult.value > 0.0 ? 1 : -1;
    return ExactArithmetic3D::expansionSign(exact(point0, point1, point2, point3));
}

} // namespace Meshing
