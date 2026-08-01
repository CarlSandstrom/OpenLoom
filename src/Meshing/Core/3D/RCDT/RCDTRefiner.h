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

    /// Per-vertex insertion radius: the distance to the nearest other node at
    /// the moment this vertex was inserted. Recorded once and never updated —
    /// it is a fixed property of the vertex, not a measure of its current
    /// surroundings. Used by the proximity guard in refineStep() to bound how
    /// close a new Steiner point may land to an existing one, relative to how
    /// fine the mesh already was when that existing point was created. This
    /// is what makes each generation of insertions provably smaller than the
    /// last by a bounded factor, guaranteeing termination.
    std::unordered_map<size_t, double> insertionRadius_;

    /// Seeds insertionRadius_ for every node present before refinement starts
    /// (the initial boundary/corner discretization and the supertet corners).
    void initializeInsertionRadii();

    /// Distance from point to its nearest existing node. Used both to check
    /// the insertion-radius guard and, after a successful insertion, to seed
    /// the new node's own entry in insertionRadius_.
    double computeNearestNeighborDistance(const Point3D& point) const;

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
