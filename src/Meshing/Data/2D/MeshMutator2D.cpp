#include "MeshMutator2D.h"

#include "Node2D.h"

namespace Meshing
{

MeshMutator2D::MeshMutator2D(MeshData2D& meshData) :
    meshData_(meshData)
{
}

size_t MeshMutator2D::addNode(const Point2D& coordinates)
{
    auto node = std::make_unique<Node2D>(coordinates);
    size_t id = meshData_.addNodeInternal(std::move(node));
    return id;
}

void MeshMutator2D::moveNode(size_t id, const Point2D& newCoords)
{
    Node2D* node = meshData_.getNodeMutable(id);
    if (node != nullptr)
    {
        node->setCoordinates(newCoords);
    }
}

void MeshMutator2D::removeNode(size_t id)
{
    meshData_.removeNodeInternal(id);
}

size_t MeshMutator2D::addElement(std::unique_ptr<IElement> element)
{
    size_t id = meshData_.addElementInternal(std::move(element));
    return id;
}

void MeshMutator2D::removeElement(size_t id)
{
    meshData_.removeElementInternal(id);
}

} // namespace Meshing
