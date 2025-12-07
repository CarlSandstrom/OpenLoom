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

void MeshData2D::addNodeInternal_(size_t id, std::unique_ptr<Node2D> node)
{
    nodes_[id] = std::move(node);
}

void MeshData2D::addElementInternal_(size_t id, std::unique_ptr<IElement> element)
{
    elements_[id] = std::move(element);
}

void MeshData2D::removeNodeInternal_(size_t id)
{
    nodes_.erase(id);
}

void MeshData2D::removeElementInternal_(size_t id)
{
    elements_.erase(id);
}

Node2D* MeshData2D::getNodeMutable_(size_t id)
{
    auto it = nodes_.find(id);
    return (it != nodes_.end()) ? it->second.get() : nullptr;
}

} // namespace Meshing
