#include "MeshData2D.h"

namespace Meshing
{

const Node2D* MeshData2D::getNode(size_t id) const
{
    auto it = nodes_.find(id);
    return (it != nodes_.end()) ? it->second.get() : nullptr;
}

const IElement* MeshData2D::getElement(size_t id) const
{
    auto it = elements_.find(id);
    return (it != elements_.end()) ? it->second.get() : nullptr;
}

size_t MeshData2D::addNodeInternal(std::unique_ptr<Node2D> node)
{
    nodes_[nextNodeId_] = std::move(node);
    return nextNodeId_++;
}

size_t MeshData2D::addElementInternal(std::unique_ptr<IElement> element)
{
    elements_[nextElementId_] = std::move(element);
    return nextElementId_++;
}

void MeshData2D::removeNodeInternal(size_t id)
{
    nodes_.erase(id);
}

void MeshData2D::removeElementInternal(size_t id)
{
    elements_.erase(id);
}

Node2D* MeshData2D::getNodeMutable(size_t id)
{
    auto it = nodes_.find(id);
    return (it != nodes_.end()) ? it->second.get() : nullptr;
}

} // namespace Meshing
