#include "MeshOperations2D.h"

#include "Node2D.h"

namespace Meshing
{

MeshOperations2D::MeshOperations2D(MeshData2D& meshData) :
    meshData_(meshData)
{
}

size_t MeshOperations2D::addNode(const Point2D& coordinates)
{
    size_t id = nextNodeId_++;
    auto node = std::make_unique<Node2D>(coordinates);
    meshData_.addNodeInternal_(id, std::move(node));
    return id;
}

void MeshOperations2D::moveNode(size_t id, const Point2D& newCoords)
{
    Node2D* node = meshData_.getNodeMutable_(id);
    if (node != nullptr)
    {
        node->setCoordinates(newCoords);
    }
}

void MeshOperations2D::removeNode(size_t id)
{
    meshData_.removeNodeInternal_(id);
}

size_t MeshOperations2D::addElement(std::unique_ptr<IElement> element)
{
    size_t id = nextElementId_++;
    meshData_.addElementInternal_(id, std::move(element));
    return id;
}

void MeshOperations2D::removeElement(size_t id)
{
    meshData_.removeElementInternal_(id);
}

} // namespace Meshing
