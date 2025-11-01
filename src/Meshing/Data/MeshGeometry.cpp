#include "MeshGeometry.h"
#include <stdexcept>

namespace Meshing
{

MeshGeometry::MeshGeometry()
{
}

const std::unordered_map<size_t, std::unique_ptr<Node>>& MeshGeometry::getNodes() const
{
    return nodes_;
}

const std::unordered_map<size_t, std::unique_ptr<Element>>& MeshGeometry::getElements() const
{
    return elements_;
}

const Node* MeshGeometry::getNode(size_t id) const
{
    auto it = nodes_.find(id);
    return (it != nodes_.end()) ? it->second.get() : nullptr;
}

const Element* MeshGeometry::getElement(size_t id) const
{
    auto it = elements_.find(id);
    return (it != elements_.end()) ? it->second.get() : nullptr;
}

size_t MeshGeometry::getNodeCount() const
{
    return nodes_.size();
}

size_t MeshGeometry::getElementCount() const
{
    return elements_.size();
}

void MeshGeometry::addNodeInternal_(size_t id, std::unique_ptr<Node> node)
{
    nodes_[id] = std::move(node);
}

void MeshGeometry::addElementInternal_(size_t id, std::unique_ptr<Element> element)
{
    elements_[id] = std::move(element);
}

void MeshGeometry::removeNodeInternal_(size_t id)
{
    nodes_.erase(id);
}

void MeshGeometry::removeElementInternal_(size_t id)
{
    elements_.erase(id);
}

Node* MeshGeometry::getNodeMutable_(size_t id)
{
    auto it = nodes_.find(id);
    return (it != nodes_.end()) ? it->second.get() : nullptr;
}

} // namespace Meshing