#pragma once

#include "Common/Types.h"
#include "Meshing/Connectivity/EdgeKey.h"
#include "Meshing/Connectivity/FaceKey.h"
#include "Meshing/Core/3D/RCDT/SurfaceProjector.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"

#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace Meshing
{

class MeshingContext3D;
class RestrictedTriangulation;
class RCDTTetQualityController;

/// Refines a restricted Delaunay triangulation in ambient 3D space.
///
/// Each refineStep() makes at most one insertion, for the first priority that
/// has work:
///
///   1. Split an encroached curve segment.
///   2. Insert a point for a bad restricted triangle (circumradius/edge or
///      chord deviation).
///   3. Insert the circumcenter of a skinny tetrahedron -- only when a
///      RCDTTetQualityController was supplied, i.e. when meshing a volume.
///   4. Repair a non-manifold edge of the restricted-face set. If the edge's
///      endpoints are joined by a curve segment, that segment is split, which
///      keeps the new point exactly on the crease; projecting onto one of the
///      surfaces instead was measured to move such a defect, not resolve it.
///
/// Within a priority, candidates are taken in the order their containers yield
/// them, not worst first, so that order shapes the output mesh.
///
/// Priorities 2-4 demote to splitting a curve segment when their point would
/// encroach one (Shewchuk), and never insert within minimumEdgeLength_ of an
/// existing node. No priority inserts inside a protecting ball (see
/// encroachesProtectingBall()). A candidate refused for any of these reasons
/// is recorded as unrefinable and not tried again.
///
/// Slivers -- tetrahedra with an acceptable circumradius/edge ratio but poor
/// dihedral angles -- are not detected, and a near-flat tetrahedron whose
/// circumcenter lies far outside the mesh is left unrefined (see the
/// circumradius guard in refineBadTetrahedra()). Measured to affect a few
/// percent of tetrahedra.
class RCDTRefiner
{
public:
    RCDTRefiner(MeshingContext3D& context,
                RestrictedTriangulation& restrictedTriangulation,
                const SurfaceMesh3DQualitySettings& settings,
                double minimumEdgeLength,
                const RCDTTetQualityController* tetQualityController = nullptr);

    void refine();

private:
    MeshingContext3D* context_;
    RestrictedTriangulation* restrictedTriangulation_;
    SurfaceMesh3DQualitySettings settings_;
    const RCDTTetQualityController* tetQualityController_;
    SurfaceProjector surfaceProjector_;

    /// Candidates priorities 2-4 have given up on; never tried again, and
    /// never cleared. Clearing them after every segment split was measured on
    /// SaddleSurfaceMesh to rediscover the same unfixable candidates about
    /// 100 times over, with no gain. Entries are keyed by nodes or element ID,
    /// so a candidate an insertion restructures returns under a new key; one
    /// whose key survives stays blocked even if its insertion point has moved.
    std::unordered_set<FaceKey, FaceKeyHash> unrefinableTriangles_;
    std::unordered_set<size_t> unrefinableTetrahedra_;
    std::unordered_set<EdgeKey, EdgeKeyHash> unrefinableNonManifoldEdges_;

    /// Curve segments that are not split again: at or below
    /// minimumEdgeLength_, or declined by trySplitSegment(). Without the size
    /// floor, a segment near a small input angle is bisected forever, because
    /// each half is encroached again by the same nearby vertex.
    std::unordered_set<size_t> unrefinableSegments_;

    /// Curve segments currently encroached by some node. Seeded once in
    /// refine(), then kept current by each insertion (see
    /// updateEncroachedSegmentsForNewNode() and splitSegment()) rather than
    /// rescanned every step, which cost O(nodes x segments) per step.
    /// Priority 1 iterates it, so its iteration order shapes the output.
    std::unordered_set<size_t> encroachedSegments_;

    /// Node positions for the current refineStep(), built on first use by
    /// getNodePositionMap() and reset at the start of each step. It may be
    /// built before or after the step's one insertion: the only lookup after
    /// an insertion skips segments ending at the new node, so both give the
    /// same answer.
    std::optional<std::unordered_map<size_t, Point3D>> cachedNodePositionMap_;

    /// Size floor (see MinimumEdgeLengthEstimator). Bounds how short a segment,
    /// restricted triangle or non-manifold edge may get before it is left
    /// unrefined, how close an insertion may land to an existing node, and how
    /// large a tetrahedron's circumradius may be.
    double minimumEdgeLength_;

    /// Performs one refinement step. Returns true if any insertion was made.
    bool refineStep();

    /// Priority 3: inserts the circumcenter of the first refinable skinny
    /// tetrahedron, or splits the segment it would encroach. Returns false
    /// without doing anything when no tetQualityController_ was supplied.
    /// Returns true if an insertion or split was made.
    bool refineBadTetrahedra();

    /// Priority 3, then priority 4 if priority 3 found nothing to do.
    bool refineRemainingPriorities();

    /// Priority 4: repairs the first refinable non-manifold restricted-face
    /// edge (see RestrictedTriangulation::findNonManifoldEdges()) by splitting
    /// the curve segment joining its endpoints if there is one, otherwise by
    /// inserting its midpoint projected onto the defect's surface. Returns true
    /// if an insertion or split was made.
    bool refineNonManifoldEdges();

    /// Inserts point into the Delaunay and updates RestrictedTriangulation.
    /// Pre-computes cavity interior faces before insertion so the restricted
    /// triangulation can remove stale faces incrementally.
    /// Returns the new node ID.
    size_t insertAndUpdate(const Point3D& point, const std::vector<std::string>& geometryIds);

    /// Splits a curve segment at its arc-length midpoint.
    /// Inserts the new node via Bowyer-Watson and updates RestrictedTriangulation.
    /// Returns true on success.
    bool splitSegment(size_t segmentId);

    /// The only route to splitSegment(). Declines -- marking segmentId
    /// unrefinable and returning false -- when the geometry or edge is missing
    /// or the split point lies inside a protecting ball; callers then move on
    /// to their next candidate. The ball check matters even though the point
    /// is on the curve: inside an unrelated ball (e.g. a nearby corner's) the
    /// point is hidden, Bowyer-Watson finds no conflicting tetrahedra, and the
    /// node is added unconnected to the mesh -- the orphaned edge nodes
    /// RCDTMesherCylinderTest.AllEdgeNodesCovered caught.
    bool trySplitSegment(size_t segmentId);

    /// Builds a node-ID → position lookup from the current mesh.
    std::unordered_map<size_t, Point3D> buildNodePositionMap() const;

    /// Returns cachedNodePositionMap_, building it on first access since the
    /// last reset (see its member doc) rather than every call.
    const std::unordered_map<size_t, Point3D>& getNodePositionMap();

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

    /// True if point lies strictly inside the protecting ball of a node with a
    /// positive weight (see Node3D::getWeight() and CurveProtectionScheme,
    /// OPE-176). An insertion there would erode the crease protection the
    /// ball exists for. A ball cannot be split the way an encroached segment
    /// can, so the candidate is marked unrefinable instead.
    bool encroachesProtectingBall(const Point3D& point) const;

    /// Returns the FaceKeys of faces shared by exactly two of conflictingTets:
    /// the cavity's interior faces, as the conflict set stands before insertion.
    std::vector<FaceKey> computeCavityInteriorFaces(
        const std::vector<size_t>& conflictingTets) const;
};

} // namespace Meshing
