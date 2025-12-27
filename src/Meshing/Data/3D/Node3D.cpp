#include "Node3D.h"

namespace Meshing
{

Node3D::Node3D(const Point3D& coordinates) :
    coordinates_(coordinates), isBoundary_(false), geometryId_("")
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
    return isBoundary_;
}

void Node3D::setBoundary(bool boundary)
{
    isBoundary_ = boundary;
}

void Node3D::setGeometryId(const std::string& id)
{
    geometryId_ = id;
}

const std::string& Node3D::getGeometryId() const
{
    return geometryId_;
}

} // namespace Meshing