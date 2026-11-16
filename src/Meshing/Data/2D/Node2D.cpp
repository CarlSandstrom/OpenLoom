#include "Meshing/Data/2D/Node2D.h"

namespace Meshing
{

Node2D::Node2D(const Point2D& coordinates) :
    coordinates_(coordinates)
{
}

const Point2D& Node2D::getCoordinates() const
{
    return coordinates_;
}

} // namespace Meshing
