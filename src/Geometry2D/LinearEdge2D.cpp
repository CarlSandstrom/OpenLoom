#include "LinearEdge2D.h"

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

Meshing::Point2D LinearEdge2D::getPoint(double t) const
{
    // Linear interpolation: P(t) = (1-t)*start + t*end for t in [0,1]
    double u = (1.0 - t) * startPoint_.x() + t * endPoint_.x();
    double v = (1.0 - t) * startPoint_.y() + t * endPoint_.y();
    return Meshing::Point2D(u, v);
}

double LinearEdge2D::getLength() const
{
    double du = endPoint_.x() - startPoint_.x();
    double dv = endPoint_.y() - startPoint_.y();
    return std::sqrt(du * du + dv * dv);
}

} // namespace Geometry2D
