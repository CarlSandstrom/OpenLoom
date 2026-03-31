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
    nodeGeometryIds_.erase(id);
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

const std::vector<std::string>& MeshData2D::getGeometryIds(size_t nodeId) const
{
    static const std::vector<std::string> empty;
    auto it = nodeGeometryIds_.find(nodeId);
    return it != nodeGeometryIds_.end() ? it->second : empty;
}

bool MeshData2D::isBoundaryNode(size_t nodeId) const
{
    auto it = nodeGeometryIds_.find(nodeId);
    return it != nodeGeometryIds_.end() && !it->second.empty();
}

void MeshData2D::setNodeGeometryIdsInternal(size_t nodeId, std::vector<std::string> ids)
{
    nodeGeometryIds_[nodeId] = std::move(ids);
}

size_t MeshData2D::addCurveSegmentInternal(const CurveSegment& segment)
{
    return curveSegmentManager_.addSegment(segment);
}

void MeshData2D::setCurveSegmentManagerInternal(CurveSegmentManager manager)
{
    curveSegmentManager_ = std::move(manager);
}

std::pair<size_t, size_t> MeshData2D::splitCurveSegmentInternal(size_t nodeId1, size_t nodeId2,
                                                                  size_t newNodeId, double tMid)
{
    auto segmentIdOpt = curveSegmentManager_.findSegmentId(nodeId1, nodeId2);
    if (!segmentIdOpt)
    {
        return {0, 0};
    }
    return curveSegmentManager_.splitAt(*segmentIdOpt, newNodeId, tMid);
}

void MeshData2D::clearCurveSegmentsInternal()
{
    curveSegmentManager_.clear();
}

} // namespace Meshing
