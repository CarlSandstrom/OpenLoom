#include "MeshData3D.h"
#include <stdexcept>

namespace Meshing
{

MeshData3D::MeshData3D()
{
}

const std::unordered_map<size_t, std::unique_ptr<Node3D>>& MeshData3D::getNodes() const
{
    return nodes_;
}

const std::unordered_map<size_t, std::unique_ptr<IElement>>& MeshData3D::getElements() const
{
    return elements_;
}

const Node3D* MeshData3D::getNode(size_t id) const
{
    auto it = nodes_.find(id);
    return (it != nodes_.end()) ? it->second.get() : nullptr;
}

const IElement* MeshData3D::getElement(size_t id) const
{
    auto it = elements_.find(id);
    return (it != elements_.end()) ? it->second.get() : nullptr;
}

size_t MeshData3D::getNodeCount() const
{
    return nodes_.size();
}

size_t MeshData3D::getElementCount() const
{
    return elements_.size();
}

void MeshData3D::addNodeInternal(size_t id, std::unique_ptr<Node3D> node)
{
    nodes_[id] = std::move(node);
}

void MeshData3D::addElementInternal(size_t id, std::unique_ptr<IElement> element)
{
    elements_[id] = std::move(element);
}

void MeshData3D::removeNodeInternal(size_t id)
{
    nodes_.erase(id);
}

void MeshData3D::removeElementInternal(size_t id)
{
    elements_.erase(id);
}

Node3D* MeshData3D::getNodeMutable(size_t id)
{
    auto it = nodes_.find(id);
    return (it != nodes_.end()) ? it->second.get() : nullptr;
}

} // namespace Meshing