#include "OpenCascade2DCorner.h"
#include <sstream>

namespace Geometry2D
{

OpenCascade2DCorner::OpenCascade2DCorner(const gp_Pnt2d& point,
                                         const std::string& id) :
    point_(point),
    id_(id)
{
    if (id_.empty())
    {
        std::ostringstream oss;
        oss << "OpenCascade2DCorner_" << std::hex
            << reinterpret_cast<uintptr_t>(&point_);
        id_ = oss.str();
    }
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
