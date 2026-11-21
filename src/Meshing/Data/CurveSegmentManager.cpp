#include "CurveSegmentManager.h"

#include "Common/Exceptions/MeshException.h"

namespace Meshing
{

SegmentId CurveSegmentManager::addSegment(const CurveSegment& segment)
{
    return addSegmentInternal(segment);
}

std::pair<SegmentId, SegmentId> CurveSegmentManager::splitAt(SegmentId segmentId, NodeId newNodeId, double midParameter)
{
    auto it = segments_.find(segmentId);
    if (it == segments_.end())
    {
        OPENLOOM_THROW_CODE(OpenLoom::MeshException,
                            OpenLoom::MeshException::ErrorCode::INVALID_OPERATION,
                            "CurveSegment with ID " + std::to_string(segmentId.value()) + " not found");
    }

    CurveSegment original = it->second;
    removeSegmentInternal(segmentId);

    CurveSegment segment1{original.startNodeId, newNodeId, original.edgeId, original.startParameter, midParameter, original.role};
    CurveSegment segment2{newNodeId, original.endNodeId, original.edgeId, midParameter, original.endParameter, original.role};

    SegmentId id1 = addSegmentInternal(segment1);
    SegmentId id2 = addSegmentInternal(segment2);

    return {id1, id2};
}

const CurveSegment& CurveSegmentManager::getSegment(SegmentId segmentId) const
{
    auto it = segments_.find(segmentId);
    if (it == segments_.end())
    {
        OPENLOOM_THROW_CODE(OpenLoom::MeshException,
                            OpenLoom::MeshException::ErrorCode::INVALID_OPERATION,
                            "CurveSegment with ID " + std::to_string(segmentId.value()) + " not found");
    }
    return it->second;
}

std::optional<SegmentId> CurveSegmentManager::findSegmentId(NodeId nodeId1, NodeId nodeId2) const
{
    auto key = (nodeId1 < nodeId2) ? std::make_pair(nodeId1, nodeId2) : std::make_pair(nodeId2, nodeId1);
    auto it = endpointToSegmentId_.find(key);
    if (it == endpointToSegmentId_.end())
        return std::nullopt;
    return it->second;
}

const std::unordered_map<SegmentId, CurveSegment>& CurveSegmentManager::getAllSegments() const
{
    return segments_;
}

std::vector<CurveSegment> CurveSegmentManager::getSegmentsForEdge(const std::string& edgeId) const
{
    std::vector<CurveSegment> result;
    for (const auto& [segmentId, segment] : segments_)
    {
        if (segment.edgeId == edgeId)
            result.push_back(segment);
    }
    std::sort(result.begin(), result.end(),
              [](const CurveSegment& a, const CurveSegment& b)
              { return a.startParameter < b.startParameter; });
    return result;
}

std::vector<NodeId> CurveSegmentManager::getOrderedNodeIdsForEdge(const std::string& edgeId) const
{
    const auto segments = getSegmentsForEdge(edgeId);
    if (segments.empty())
        return {};

    std::vector<NodeId> nodeIds;
    nodeIds.reserve(segments.size() + 1);
    nodeIds.push_back(segments.front().startNodeId);
    for (const auto& segment : segments)
        nodeIds.push_back(segment.endNodeId);
    return nodeIds;
}

size_t CurveSegmentManager::size() const
{
    return segments_.size();
}

bool CurveSegmentManager::empty() const
{
    return segments_.empty();
}

SegmentId CurveSegmentManager::addSegmentInternal(const CurveSegment& segment)
{
    SegmentId id{nextId_++};
    segments_[id] = segment;
    auto key = (segment.startNodeId < segment.endNodeId) ?
                   std::make_pair(segment.startNodeId, segment.endNodeId) :
                   std::make_pair(segment.endNodeId, segment.startNodeId);
    endpointToSegmentId_[key] = id;
    return id;
}

void CurveSegmentManager::removeSegmentInternal(SegmentId segmentId)
{
    auto it = segments_.find(segmentId);
    if (it == segments_.end())
        return;

    const auto& segment = it->second;
    auto key = (segment.startNodeId < segment.endNodeId) ?
                   std::make_pair(segment.startNodeId, segment.endNodeId) :
                   std::make_pair(segment.endNodeId, segment.startNodeId);
    endpointToSegmentId_.erase(key);
    segments_.erase(it);
}

} // namespace Meshing
