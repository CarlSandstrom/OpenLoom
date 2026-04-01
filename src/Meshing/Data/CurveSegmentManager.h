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
    size_t nodeId1 = 0;
    size_t nodeId2 = 0;
    std::string edgeId;
    double tStart = 0.0;
    double tEnd = 0.0;
    ConstraintRole role = ConstraintRole::Boundary;
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

    /// Returns the IDs of segments whose diametral sphere contains point.
    /// nodePositions maps each node ID to its 3D position.
    std::vector<size_t> findEncroached(const Point3D& point,
                                       const std::unordered_map<size_t, Point3D>& nodePositions) const;

    /// Returns all segments for a given edge ID, sorted by tStart (ascending).
    std::vector<CurveSegment> getSegmentsForEdge(const std::string& edgeId) const;

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
