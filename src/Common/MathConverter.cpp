#include "Common/MathConverter.h"

namespace Meshing
{

Point2D MathConverter::toPoint2D(const std::array<double, 2>& values)
{
    return Point2D(values[0], values[1]);
}

std::array<double, 2> MathConverter::toArray(const Point2D& point)
{
    return {point.x(), point.y()};
}

Point3D MathConverter::toPoint3D(const std::array<double, 3>& values)
{
    return Point3D(values[0], values[1], values[2]);
}

std::array<double, 3> MathConverter::toArray(const Point3D& point)
{
    return {point.x(), point.y(), point.z()};
}

} // namespace Meshing
