#include "Node3D.h"

namespace Meshing
{

Node3D::Node3D(const Point3D& coordinates) :
    coordinates_(coordinates)
{
}

const Point3D& Node3D::getCoordinates() const
{
    return coordinates_;
}

void Node3D::setCoordinates(const Point3D& coords)
{
    coordinates_ = coords;
}

double Node3D::getWeight() const
{
    return weight_;
}

void Node3D::setWeight(double weight)
{
    weight_ = weight;
}

} // namespace Meshing
