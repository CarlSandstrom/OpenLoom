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

void MeshData2D::addConstrainedSegmentInternal(const ConstrainedSegment2D& segment)
{
    constrainedSegments_.push_back(segment);
}

void MeshData2D::removeConstrainedSegmentInternal(size_t nodeId1, size_t nodeId2)
{
    std::erase_if(constrainedSegments_,
        [nodeId1, nodeId2](const ConstrainedSegment2D& seg)
        {
            return (seg.nodeId1 == nodeId1 && seg.nodeId2 == nodeId2) ||
                   (seg.nodeId1 == nodeId2 && seg.nodeId2 == nodeId1);
        });
}

void MeshData2D::replaceConstrainedSegmentInternal(const ConstrainedSegment2D& oldSegment,
                                                    const ConstrainedSegment2D& newSeg1,
                                                    const ConstrainedSegment2D& newSeg2)
{
    for (auto it = constrainedSegments_.begin(); it != constrainedSegments_.end(); ++it)
    {
        if (it->nodeId1 == oldSegment.nodeId1 && it->nodeId2 == oldSegment.nodeId2)
        {
            *it = newSeg1;
            constrainedSegments_.push_back(newSeg2);
            return;
        }
    }
    // Segment not found - this is a no-op (may happen during mesh modifications)
}

void MeshData2D::clearConstrainedSegmentsInternal()
{
    constrainedSegments_.clear();
}

} // namespace Meshing
