#pragma once

#include "Common/Types.h"

#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace Meshing
{

class MeshingContext3D;
class RestrictedTriangulation;

/// Inserts RCDTRefiner's points into the ambient tetrahedralization, keeping
/// the restricted triangulation and the set of encroached curve segments
/// current with each insertion. Every insertion refinement makes goes through
/// here: priority 1's segment splits, which iterate the encroached set this
/// class owns, and priorities 2-4's guarded insertions (see tryInsert()).
class RCDTPointInserter
{
public:
    RCDTPointInserter(MeshingContext3D& context,
                      RestrictedTriangulation& restrictedTriangulation,
                      double minimumEdgeLength);

    /// Finds every curve segment an existing node encroaches. Called once
    /// before refinement; each insertion keeps the set current from then on.
    void seedEncroachedSegments();

    /// Starts a refinement step: node positions cached during the previous
    /// step are stale, since it inserted a node.
    void beginStep();

    /// Priority 1: splits the first encroached curve segment that is above
    /// minimumEdgeLength_ and not declined by trySplitSegment(). Returns true
    /// if a segment was split.
    bool splitEncroachedSegment();

    /// Inserts point on behalf of priorities 2-4 unless a guard refuses it.
    /// A point within minimumEdgeLength_ of an existing node is refused; one
    /// that would encroach a curve segment splits that segment instead
    /// (Shewchuk's demotion); one inside a protecting ball is refused. Returns
    /// true if point or the demoted split was inserted. False means the
    /// candidate has no other way to be refined, and the caller marks it
    /// unrefinable.
    bool tryInsert(const Point3D& point, const std::vector<std::string>& geometryIds);

    /// The only route to splitSegment(). Declines -- marking segmentId
    /// unrefinable and returning false -- when the geometry or edge is missing
    /// or the split point lies inside a protecting ball; callers then move on
    /// to their next candidate. The ball check matters even though the point
    /// is on the curve: inside an unrelated ball (e.g. a nearby corner's) the
    /// point is hidden, Bowyer-Watson finds no conflicting tetrahedra, and the
    /// node is added unconnected to the mesh -- the orphaned edge nodes
    /// RCDTMesherCylinderTest.AllEdgeNodesCovered caught.
    bool trySplitSegment(size_t segmentId);

    /// Returns cachedNodePositionMap_, building it on first access since the
    /// last beginStep() (see its member doc) rather than every call.
    const std::unordered_map<size_t, Point3D>& getNodePositionMap();

private:
    MeshingContext3D* context_;
    RestrictedTriangulation* restrictedTriangulation_;

    /// Size floor (see MinimumEdgeLengthEstimator): segments at or below it are
    /// not split, and no point is inserted closer than it to an existing node.
    double minimumEdgeLength_;

    /// Curve segments that are not split again: at or below
    /// minimumEdgeLength_, or declined by trySplitSegment(). Without the size
    /// floor, a segment near a small input angle is bisected forever, because
    /// each half is encroached again by the same nearby vertex.
    std::unordered_set<size_t> unrefinableSegments_;

    /// Curve segments currently encroached by some node. Seeded once by
    /// seedEncroachedSegments(), then kept current by each insertion (see
    /// updateEncroachedSegmentsForNewNode() and splitSegment()) rather than
    /// rescanned every step, which cost O(nodes x segments) per step.
    /// Priority 1 iterates it, so its iteration order shapes the output.
    std::unordered_set<size_t> encroachedSegments_;

    /// Node positions for the current refinement step, built on first use by
    /// getNodePositionMap() and reset by beginStep(). It may be built before
    /// or after the step's one insertion: the only lookup after an insertion
    /// skips segments ending at the new node, so both give the same answer.
    std::optional<std::unordered_map<size_t, Point3D>> cachedNodePositionMap_;

    /// Inserts point into the Delaunay and updates RestrictedTriangulation.
    /// Pre-computes cavity interior faces before insertion so the restricted
    /// triangulation can remove stale faces incrementally.
    /// Returns the new node ID.
    size_t insertAndUpdate(const Point3D& point, const std::vector<std::string>& geometryIds);

    /// Splits a curve segment at its arc-length midpoint.
    /// Inserts the new node via Bowyer-Watson and updates RestrictedTriangulation.
    /// Returns true on success.
    bool splitSegment(size_t segmentId);

    /// Adds every curve segment that the new node at position encroaches to
    /// encroachedSegments_, skipping segments that end at newNodeId. Called
    /// after every insertion.
    void updateEncroachedSegmentsForNewNode(size_t newNodeId, const Point3D& position);

    /// Checks segmentId against every current node and adds it to
    /// encroachedSegments_ if any (other than its own endpoints) encroaches
    /// it. Called for each of the two new segments a split produces, since
    /// an existing, unrelated node can already sit inside a freshly-split
    /// segment's (smaller) diametral sphere.
    void checkSegmentAgainstAllNodes(size_t segmentId);
};

} // namespace Meshing
