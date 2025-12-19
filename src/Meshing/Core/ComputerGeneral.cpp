#include "ComputerGeneral.h"

namespace Meshing
{

bool ComputerGeneral::getIsPointInsideCircumscribingSphere(const CircumscribedSphere& sphere,
                                                           const Point3DRef point,
                                                           double tolerance)
{
    const double radiusSq = sphere.radius * sphere.radius;
    const double distanceSq = (point - sphere.center).squaredNorm();
    return distanceSq <= radiusSq + tolerance;
}

} // namespace Meshing
