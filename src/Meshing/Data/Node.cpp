#include "Node.h"

namespace Meshing
{

Node::Node(const std::array<double, 3>& coordinates) :
    coordinates_(coordinates), isBoundary_(false), geometryId_("")
{
}

const std::array<double, 3>& Node::getCoordinates() const
{
    return coordinates_;
}

void Node::setCoordinates(const std::array<double, 3>& coords)
{
    coordinates_ = coords;
}

bool Node::isBoundary() const
{
    return isBoundary_;
}

void Node::setBoundary(bool boundary)
{
    isBoundary_ = boundary;
}

void Node::setGeometryId(const std::string& id)
{
    geometryId_ = id;
}

const std::string& Node::getGeometryId() const
{
    return geometryId_;
}

} // namespace Meshing