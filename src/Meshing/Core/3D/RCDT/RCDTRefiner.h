#pragma once

#include "Common/Types.h"
#include "Meshing/Connectivity/FaceKey.h"
#include "Meshing/Core/3D/RCDT/SurfaceProjector.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"

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
/// Mirrors Shewchuk's three-priority structure, but priorities 2 and 3
/// operate on RCDT's own restricted-triangulation and tet-quality concepts
/// rather than his subfacet/subsegment encroachment machinery:
///
///   Priority 1 — split encroached curve segments, unless already at the
///                minimumEdgeLength_ size floor (see class member docs)
///   Priority 2 — split bad restricted triangles (circumradius/edge or chord deviation)
///   Priority 3 — split bad tetrahedra (circumradius/edge), only when a
///                RCDTTetQualityController was supplied at construction
///                (RCDTMesher::meshVolume() only — meshSurface() never pays
///                for tet-quality refinement it has no use for)
///
/// The circumcenter demotion rule (Shewchuk) ensures termination: if inserting
/// the circumcenter of a bad triangle or tetrahedron would encroach a
/// segment, the segment is split instead.
///
/// Priority 3 does not detect or fix slivers (tetrahedra with an acceptable
/// circumradius/edge ratio but poor dihedral angles, or genuinely thin/flat
/// tets whose true circumcenter recedes far outside the mesh's extent) --
/// same documented limitation as the legacy Shewchuk volume mesher this
/// replaced (removed in OPE-161). Such tets are left unrefined (see the
/// circumradius sanity guard in refineBadTetrahedra()) rather than chased
/// with an insertion point that would corrupt the mesh; confirmed
/// empirically they arise in small numbers (a few percent of tets) rather
/// than dominating.
class RCDTRefiner
{
public:
    RCDTRefiner(MeshingContext3D& context,
                RestrictedTriangulation& restrictedTriangulation,
                const SurfaceMesh3DQualitySettings& settings,
                const RCDTTetQualityController* tetQualityController = nullptr);

    void refine();

private:
    MeshingContext3D* context_;
    RestrictedTriangulation* restrictedTriangulation_;
    SurfaceMesh3DQualitySettings settings_;
    const RCDTTetQualityController* tetQualityController_;
    SurfaceProjector surfaceProjector_;
    std::unordered_set<FaceKey, FaceKeyHash> unrefinableTriangles_;
    std::unordered_set<size_t> unrefinableTetrahedra_;

    /// Curve segments (keyed by CurveSegmentManager segment ID) already at or
    /// below minimumEdgeLength_ that are still encroached. Without this,
    /// a segment pinned close to a genuinely sharp/small-angle feature (e.g.
    /// a nearby vertex whose position never changes) can be bisected
    /// forever: each split's midpoint still falls inside that vertex's
    /// encroachment sphere, so the newly created segment is "encroached"
    /// again on the very next iteration. Cleared in splitSegment(), same
    /// reasoning as the other two unrefinable sets.
    std::unordered_set<size_t> unrefinableSegments_;

    /// Resolved once at the start of refine() from settings_.minimumEdgeLength
    /// (or derived from the initial discretization if unset — see
    /// resolveMinimumEdgeLength()). A restricted triangle at or below this
    /// shortest-edge length is left unrefined even if still quality-bad.
    double minimumEdgeLength_ = 0.0;

    /// Resolves minimumEdgeLength_: settings_.minimumEdgeLength if set,
    /// otherwise the median nearest-neighbor distance among the nodes
    /// present before refinement starts (excluding the supertet corners),
    /// divided by 10. Median rather than minimum because a periodic curve's
    /// discretization can leave a short "remainder" segment near its seam
    /// vertex that isn't representative of the intended spacing.
    double resolveMinimumEdgeLength() const;

    /// Performs one refinement step. Returns true if any insertion was made.
    bool refineStep();

    /// Priority 3: find the worst-quality tetrahedron and insert its
    /// circumcenter (demoting to a curve-segment split if the circumcenter
    /// would encroach one), same shape as the restricted-triangle handling
    /// in refineStep(). No-op (returns false immediately) if
    /// tetQualityController_ is null. Returns true if any insertion/split
    /// was made.
    bool refineBadTetrahedra();

    /// Inserts point into the Delaunay and updates RestrictedTriangulation.
    /// Pre-computes cavity interior faces before insertion so the restricted
    /// triangulation can remove stale faces incrementally.
    /// Returns the new node ID.
    size_t insertAndUpdate(const Point3D& point, const std::vector<std::string>& geometryIds);

    /// Splits a curve segment at its arc-length midpoint.
    /// Inserts the new node via Bowyer-Watson and updates RestrictedTriangulation.
    /// Returns true on success.
    bool splitSegment(size_t segmentId);

    /// Builds a node-ID → position lookup from the current mesh.
    std::unordered_map<size_t, Point3D> buildNodePositionMap() const;

    /// Returns the FaceKeys of faces shared by exactly two conflicting tetrahedra
    /// (cavity interior faces that will be removed by Bowyer-Watson insertion).
    std::vector<FaceKey> computeCavityInteriorFaces(
        const std::vector<size_t>& conflictingTets) const;
};

} // namespace Meshing
