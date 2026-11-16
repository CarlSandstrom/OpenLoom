#include "Geometry/2D/Base/LinearEdge2D.h"

#include <cmath>

namespace Geometry2D
{

LinearEdge2D::LinearEdge2D(const std::string& id,
                           const Meshing::Point2D& startPoint,
                           const Meshing::Point2D& endPoint) :
    id_(id),
    startPoint_(startPoint),
    endPoint_(endPoint)
{
}

Meshing::Point2D LinearEdge2D::getPoint(double parameter) const
{
    double x = (1.0 - parameter) * startPoint_.x() + parameter * endPoint_.x();
    double y = (1.0 - parameter) * startPoint_.y() + parameter * endPoint_.y();
    return Meshing::Point2D(x, y);
}

double LinearEdge2D::getLength() const
{
    double deltaX = endPoint_.x() - startPoint_.x();
    double deltaY = endPoint_.y() - startPoint_.y();
    return std::sqrt(deltaX * deltaX + deltaY * deltaY);
}

} // namespace Geometry2D
