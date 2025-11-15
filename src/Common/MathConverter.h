#pragma once

#include <array>

#include "Common/Types.h"

namespace Meshing
{

class MathConverter
{
public:
    static Point2D toPoint2D(const std::array<double, 2>& values);
    static std::array<double, 2> toArray(const Point2D& point);
    static Point3D toPoint3D(const std::array<double, 3>& values);
    static std::array<double, 3> toArray(const Point3D& point);
};

} // namespace Meshing
