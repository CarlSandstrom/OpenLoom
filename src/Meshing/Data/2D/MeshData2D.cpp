#include "Meshing/Data/2D/MeshData2D.h"

namespace Meshing
{

const Node2D* MeshData2D::getNode(NodeId id) const
{
    auto found = nodes_.find(id);
    return found != nodes_.end() ? found->second.get() : nullptr;
}

const IElement* MeshData2D::getElement(ElementId id) const
{
    auto found = elements_.find(id);
    return found != elements_.end() ? found->second.get() : nullptr;
}

NodeId MeshData2D::addNodeInternal(std::unique_ptr<Node2D> node)
{
    nodes_[NodeId{nextNodeId_}] = std::move(node);
    return NodeId{nextNodeId_++};
}

ElementId MeshData2D::addElementInternal(std::unique_ptr<IElement> element)
{
    elements_[ElementId{nextElementId_}] = std::move(element);
    return ElementId{nextElementId_++};
}

void MeshData2D::removeElementInternal(ElementId id)
{
    elements_.erase(id);
}

void MeshData2D::setNodeGeometryIdsInternal(NodeId nodeId, std::vector<std::string> geometryIds)
{
    nodeGeometryIds_[nodeId] = std::move(geometryIds);
}

void MeshData2D::setCurveSegmentManagerInternal(CurveSegmentManager manager)
{
    curveSegmentManager_ = std::move(manager);
}

std::pair<SegmentId, SegmentId> MeshData2D::splitCurveSegmentInternal(NodeId nodeId1, NodeId nodeId2,
                                                                      NodeId newNodeId, double tMid)
{
    auto optionalSegmentId = curveSegmentManager_.findSegmentId(nodeId1, nodeId2);
    if (!optionalSegmentId) return {SegmentId{0}, SegmentId{0}};
    return curveSegmentManager_.splitAt(*optionalSegmentId, newNodeId, tMid);
}

} // namespace Meshing
