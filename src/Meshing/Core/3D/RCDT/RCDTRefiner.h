#pragma once

#include "Common/Types.h"
#include "Meshing/Connectivity/FaceKey.h"
#include "Meshing/Core/3D/RCDT/RCDTQualitySettings.h"
#include "Meshing/Core/3D/RCDT/SurfaceProjector.h"

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace Meshing
{

class MeshingContext3D;
class RestrictedTriangulation;

/// Refines a restricted Delaunay triangulation in ambient 3D space.
///
/// Mirrors the two-priority Shewchuk structure but operates on restricted
/// surface triangles rather than volume tetrahedra:
///
///   Priority 1 — split encroached curve segments
///   Priority 2 — split bad restricted triangles (circumradius/edge or chord deviation)
///
/// The circumcenter demotion rule (Shewchuk) ensures termination: if inserting
/// the circumcenter of a bad triangle would encroach a segment, the segment is
/// split instead.
class RCDTRefiner
{
public:
    RCDTRefiner(MeshingContext3D& context,
                RestrictedTriangulation& restrictedTriangulation,
                const RCDTQualitySettings& settings);

    void refine();

private:
    MeshingContext3D* context_;
    RestrictedTriangulation* restrictedTriangulation_;
    RCDTQualitySettings settings_;
    SurfaceProjector surfaceProjector_;
    std::unordered_set<FaceKey, FaceKeyHash> unrefinableTriangles_;

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
