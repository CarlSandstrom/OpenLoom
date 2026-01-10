#pragma once

#include "Common/Types.h"
#include "Meshing/Core/3D/GeometryStructures3D.h"

namespace Meshing
{

/// Static utilities for pure 3D geometric computations that don't require mesh context.
class GeometryUtilities3D
{
public:
    /// Tests if a point is inside a circumscribed sphere.
    static bool isPointInsideCircumscribingSphere(const CircumscribedSphere& sphere,
                                                  const Point3DRef point,
                                                  double tolerance = 1e-12);
};

} // namespace Meshing
