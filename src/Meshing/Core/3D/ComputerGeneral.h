#pragma once

#include "Common/Types.h"
#include "Meshing/Core/3D/GeometryStructures3D.h"

namespace Meshing
{

/// Static utilities for general geometric computations that don't require mesh context.
class ComputerGeneral
{
public:
    static bool getIsPointInsideCircumscribingSphere(const CircumscribedSphere& sphere,
                                                     const Point3DRef point,
                                                     double tolerance = 1e-12);
};

} // namespace Meshing
