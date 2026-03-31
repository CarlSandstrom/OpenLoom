#pragma once

#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>

namespace Meshing
{

enum class EdgeRole
{
    Boundary,
    Interior
};

struct CurveSegment
{
    size_t nodeId1 = 0;
    size_t nodeId2 = 0;
    std::string edgeId;
    double tStart = 0.0;
    double tEnd = 0.0;
    EdgeRole role = EdgeRole::Boundary;
};

class CurveSegmentManager
{
public:
    size_t addSegment(const CurveSegment& segment);

    /// Splits the segment identified by segmentId at tMid, inserting newNodeId between
    /// the original endpoints. Removes the original segment and adds two sub-segments.
    /// Returns the IDs of the two new segments (segment1Id, segment2Id).
    std::pair<size_t, size_t> splitAt(size_t segmentId, size_t newNodeId, double tMid);

    const CurveSegment& getSegment(size_t segmentId) const;

    std::optional<size_t> findSegmentId(size_t nodeId1, size_t nodeId2) const;

    const std::unordered_map<size_t, CurveSegment>& getAllSegments() const;

    size_t size() const;
    bool empty() const;
    void clear();

private:
    size_t addSegmentInternal(const CurveSegment& segment);
    void removeSegmentInternal(size_t segmentId);

    std::unordered_map<size_t, CurveSegment> segments_;
    std::map<std::pair<size_t, size_t>, size_t> endpointToSegmentId_;
    size_t nextId_ = 0;
};

} // namespace Meshing
