#pragma once

#include "Common/Types.h"

namespace Meshing
{

/// Static utilities for general geometric computations that don't require mesh context.
class ComputerGeneral
{
public:
    struct CircumscribedSphere
    {
        Point3D center;
        double radius;
    };

    static bool getIsPointInsideCircumscribingSphere(const CircumscribedSphere& sphere,
                                                     const Point3DRef point,
                                                     double tolerance = 1e-12);
};

} // namespace Meshing
