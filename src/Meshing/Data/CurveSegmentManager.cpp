#include "CurveSegmentManager.h"

#include "Common/Exceptions/MeshException.h"

namespace Meshing
{

size_t CurveSegmentManager::addSegment(const CurveSegment& segment)
{
    return addSegmentInternal(segment);
}

std::pair<size_t, size_t> CurveSegmentManager::splitAt(size_t segmentId, size_t newNodeId, double tMid)
{
    auto it = segments_.find(segmentId);
    if (it == segments_.end())
    {
        OPENLOOM_THROW_CODE(OpenLoom::MeshException,
                            OpenLoom::MeshException::ErrorCode::INVALID_OPERATION,
                            "CurveSegment with ID " + std::to_string(segmentId) + " not found");
    }

    CurveSegment original = it->second;
    removeSegmentInternal(segmentId);

    CurveSegment segment1{original.nodeId1, newNodeId, original.edgeId, original.tStart, tMid, original.role};
    CurveSegment segment2{newNodeId, original.nodeId2, original.edgeId, tMid, original.tEnd, original.role};

    size_t id1 = addSegmentInternal(segment1);
    size_t id2 = addSegmentInternal(segment2);

    return {id1, id2};
}

const CurveSegment& CurveSegmentManager::getSegment(size_t segmentId) const
{
    auto it = segments_.find(segmentId);
    if (it == segments_.end())
    {
        OPENLOOM_THROW_CODE(OpenLoom::MeshException,
                            OpenLoom::MeshException::ErrorCode::INVALID_OPERATION,
                            "CurveSegment with ID " + std::to_string(segmentId) + " not found");
    }
    return it->second;
}

std::optional<size_t> CurveSegmentManager::findSegmentId(size_t nodeId1, size_t nodeId2) const
{
    auto key = (nodeId1 < nodeId2) ? std::make_pair(nodeId1, nodeId2) : std::make_pair(nodeId2, nodeId1);
    auto it = endpointToSegmentId_.find(key);
    if (it == endpointToSegmentId_.end())
        return std::nullopt;
    return it->second;
}

const std::unordered_map<size_t, CurveSegment>& CurveSegmentManager::getAllSegments() const
{
    return segments_;
}

size_t CurveSegmentManager::size() const
{
    return segments_.size();
}

bool CurveSegmentManager::empty() const
{
    return segments_.empty();
}

void CurveSegmentManager::clear()
{
    segments_.clear();
    endpointToSegmentId_.clear();
    nextId_ = 0;
}

size_t CurveSegmentManager::addSegmentInternal(const CurveSegment& segment)
{
    size_t id = nextId_++;
    segments_[id] = segment;
    auto key = (segment.nodeId1 < segment.nodeId2) ?
                   std::make_pair(segment.nodeId1, segment.nodeId2) :
                   std::make_pair(segment.nodeId2, segment.nodeId1);
    endpointToSegmentId_[key] = id;
    return id;
}

void CurveSegmentManager::removeSegmentInternal(size_t segmentId)
{
    auto it = segments_.find(segmentId);
    if (it == segments_.end())
        return;

    const auto& segment = it->second;
    auto key = (segment.nodeId1 < segment.nodeId2) ?
                   std::make_pair(segment.nodeId1, segment.nodeId2) :
                   std::make_pair(segment.nodeId2, segment.nodeId1);
    endpointToSegmentId_.erase(key);
    segments_.erase(it);
}

} // namespace Meshing
