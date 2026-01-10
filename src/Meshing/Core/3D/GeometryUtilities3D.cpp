#include "GeometryUtilities3D.h"

namespace Meshing
{

bool GeometryUtilities3D::isPointInsideCircumscribingSphere(const CircumscribedSphere& sphere,
                                                           const Point3DRef point,
                                                           double tolerance)
{
    const double radiusSq = sphere.radius * sphere.radius;
    const double distanceSq = (point - sphere.center).squaredNorm();
    return distanceSq <= radiusSq + tolerance;
}

} // namespace Meshing
