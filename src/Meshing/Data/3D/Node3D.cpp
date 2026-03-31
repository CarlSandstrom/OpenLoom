#include "Node3D.h"

namespace Meshing
{

Node3D::Node3D(const Point3D& coordinates) :
    coordinates_(coordinates)
{
}

Node3D::Node3D(const Point3D& coordinates,
               const std::vector<std::string>& geometryIds) :
    coordinates_(coordinates),
    geometryIds_(geometryIds)
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

bool Node3D::isBoundary() const
{
    return !geometryIds_.empty();
}

const std::vector<std::string>& Node3D::getGeometryIds() const
{
    return geometryIds_;
}

void Node3D::addGeometryId(const std::string& id)
{
    geometryIds_.push_back(id);
}


} // namespace Meshing