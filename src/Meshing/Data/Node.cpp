#include "Node.h"

namespace Meshing
{

Node::Node(size_t id, const std::array<double, 3>& coordinates) :
    id_(id), coordinates_(coordinates), isBoundary_(false), geometryId_("")
{
}

size_t Node::getId() const
{
    return id_;
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