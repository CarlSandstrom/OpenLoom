#include "Geometry/3D/Base/ISurface3D.h"

namespace Geometry3D
{

bool ISurface3D::isPointWithinTrimmedBoundary(const Meshing::Point3D& /*point*/) const
{
    return true;
}

} // namespace Geometry3D
