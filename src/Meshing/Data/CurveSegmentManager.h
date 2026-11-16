#pragma once

#include "Common/Types.h"
#include "Meshing/Data/ConstraintRole.h"

#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace Meshing
{

struct CurveSegment
{
    NodeId nodeId1{0};
    NodeId nodeId2{0};
    std::string edgeId;
    double tStart = 0.0;
    double tEnd = 0.0;
    ConstraintRole role = ConstraintRole::Boundary;
};

class CurveSegmentManager
{
public:
    SegmentId addSegment(const CurveSegment& segment);

    /// Splits the segment identified by segmentId at tMid, inserting newNodeId between
    /// the original endpoints. Removes the original segment and adds two sub-segments.
    /// Returns the IDs of the two new segments (segment1Id, segment2Id).
    std::pair<SegmentId, SegmentId> splitAt(SegmentId segmentId, NodeId newNodeId, double tMid);

    const CurveSegment& getSegment(SegmentId segmentId) const;

    std::optional<SegmentId> findSegmentId(NodeId nodeId1, NodeId nodeId2) const;

    const std::unordered_map<SegmentId, CurveSegment>& getAllSegments() const;

    /// Returns all segments for a given edge ID, sorted by tStart (ascending).
    std::vector<CurveSegment> getSegmentsForEdge(const std::string& edgeId) const;

    /// Returns the node chain along a given edge ID in curve order, both
    /// endpoints included. Empty if the edge has no segments.
    std::vector<NodeId> getOrderedNodeIdsForEdge(const std::string& edgeId) const;

    size_t size() const;
    bool empty() const;

private:
    SegmentId addSegmentInternal(const CurveSegment& segment);
    void removeSegmentInternal(SegmentId segmentId);

    std::unordered_map<SegmentId, CurveSegment> segments_;
    std::map<std::pair<NodeId, NodeId>, SegmentId> endpointToSegmentId_;
    size_t nextId_ = 0;
};

} // namespace Meshing
