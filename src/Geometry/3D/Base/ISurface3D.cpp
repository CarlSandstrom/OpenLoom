#include "Geometry/3D/Base/ISurface3D.h"

namespace Geometry3D
{

std::optional<PrincipalCurvatures> ISurface3D::getPrincipalCurvatures(double /*u*/,
                                                                      double /*v*/) const
{
    return std::nullopt;
}

bool ISurface3D::isPointWithinTrimmedBoundary(const Meshing::Point3D& /*point*/) const
{
    return true;
}

bool ISurface3D::isUVWithinTrimmedBoundary(double u, double v) const
{
    return isPointWithinTrimmedBoundary(getPoint(u, v));
}

} // namespace Geometry3D
