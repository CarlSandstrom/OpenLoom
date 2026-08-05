#include "Geometry/3D/Base/ISurface3D.h"

namespace Geometry3D
{

bool ISurface3D::isPointWithinTrimmedBoundary(const Meshing::Point3D& /*point*/) const
{
    return true;
}

bool ISurface3D::isUVWithinTrimmedBoundary(double u, double v) const
{
    return isPointWithinTrimmedBoundary(getPoint(u, v));
}

} // namespace Geometry3D
