#include "Meshing/Data/3D/MeshData3D.h"

namespace Meshing
{

const std::unordered_map<NodeId, std::unique_ptr<Node3D>>& MeshData3D::getNodes() const
{
    return nodes_;
}

const std::unordered_map<ElementId, std::unique_ptr<IElement>>& MeshData3D::getElements() const
{
    return elements_;
}

const Node3D* MeshData3D::getNode(NodeId id) const
{
    auto found = nodes_.find(id);
    return (found != nodes_.end()) ? found->second.get() : nullptr;
}

const IElement* MeshData3D::getElement(ElementId id) const
{
    auto found = elements_.find(id);
    return (found != elements_.end()) ? found->second.get() : nullptr;
}

size_t MeshData3D::getNodeCount() const
{
    return nodes_.size();
}

size_t MeshData3D::getElementCount() const
{
    return elements_.size();
}

void MeshData3D::addNodeInternal(NodeId id, std::unique_ptr<Node3D> node)
{
    nodes_[id] = std::move(node);
}

void MeshData3D::addElementInternal(ElementId id, std::unique_ptr<IElement> element)
{
    elements_[id] = std::move(element);
}

void MeshData3D::removeNodeInternal(NodeId id)
{
    nodes_.erase(id);
    nodeGeometryIds_.erase(id);
}

void MeshData3D::removeElementInternal(ElementId id)
{
    elements_.erase(id);
}

Node3D* MeshData3D::getNodeMutable(NodeId id)
{
    auto found = nodes_.find(id);
    return (found != nodes_.end()) ? found->second.get() : nullptr;
}

const std::vector<std::string>& MeshData3D::getGeometryIds(NodeId nodeId) const
{
    static const std::vector<std::string> empty;
    auto found = nodeGeometryIds_.find(nodeId);
    return found != nodeGeometryIds_.end() ? found->second : empty;
}

void MeshData3D::setNodeGeometryIdsInternal(NodeId nodeId, std::vector<std::string> geometryIds)
{
    nodeGeometryIds_[nodeId] = std::move(geometryIds);
}

const std::optional<std::array<NodeId, 4>>& MeshData3D::getBoundingNodeIds() const
{
    return boundingNodeIds_;
}

void MeshData3D::setBoundingNodeIdsInternal(const std::array<NodeId, 4>& boundingNodeIds)
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