#include "OpenCascade2DCorner.h"

namespace Geometry2D
{

OpenCascade2DCorner::OpenCascade2DCorner(const gp_Pnt2d& point,
                                         const std::string& id) :
    point_(point),
    id_(id)
{
}

Meshing::Point2D OpenCascade2DCorner::getPoint() const
{
    return Meshing::Point2D(point_.X(), point_.Y());
}

std::string OpenCascade2DCorner::getId() const
{
    return id_;
}

} // namespace Geometry2D
