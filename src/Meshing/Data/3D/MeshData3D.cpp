#include "MeshData3D.h"
#include <algorithm>
#include <array>
#include <ranges>
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
    nodeGeometryIds_.erase(id);
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

const std::vector<std::string>& MeshData3D::getGeometryIds(size_t nodeId) const
{
    static const std::vector<std::string> empty;
    auto it = nodeGeometryIds_.find(nodeId);
    return it != nodeGeometryIds_.end() ? it->second : empty;
}

void MeshData3D::setNodeGeometryIdsInternal(size_t nodeId, std::vector<std::string> ids)
{
    nodeGeometryIds_[nodeId] = std::move(ids);
}

const std::optional<std::array<size_t, 4>>& MeshData3D::getBoundingNodeIds() const
{
    return boundingNodeIds_;
}

void MeshData3D::setBoundingNodeIdsInternal(const std::array<size_t, 4>& boundingNodeIds)
{
    boundingNodeIds_ = boundingNodeIds;
}

void MeshData3D::clearBoundingNodeIdsInternal()
{
    boundingNodeIds_.reset();
}

const CurveSegmentManager& MeshData3D::getCurveSegmentManager() const
{
    return curveSegmentManager_;
}

} // namespace Meshing