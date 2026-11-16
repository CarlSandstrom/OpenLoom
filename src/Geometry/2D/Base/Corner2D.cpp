#include "Geometry/2D/Base/Corner2D.h"

namespace Geometry2D
{

Corner2D::Corner2D(const std::string& id, const Meshing::Point2D& point) :
    id_(id),
    point_(point)
{
}

Meshing::Point2D Corner2D::getPoint() const
{
    return point_;
}

std::string Corner2D::getId() const
{
    return id_;
}

} // namespace Geometry2D
