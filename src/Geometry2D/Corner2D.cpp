#include "Corner2D.h"

namespace Geometry2D
{

Corner2D::Corner2D(const std::string& id, const Meshing::Point2D& point) :
    id_(id),
    point_(point)
{
}

} // namespace Geometry2D
