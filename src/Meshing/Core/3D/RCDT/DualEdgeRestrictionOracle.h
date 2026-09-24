#pragma once

#include "Common/Types.h"
#include "Meshing/Connectivity/FaceKey.h"
#include "Meshing/Connectivity/TetrahedronKey.h"
#include "Meshing/Core/3D/RCDT/RestrictedFaceTypes.h"
#include "Meshing/Core/3D/RCDT/SurfaceTessellation.h"

#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace Meshing
{
class MeshConnectivity;
class MeshData3D;
class SurfaceCandidates;
class TetrahedralElement;
} // namespace Meshing

namespace Geometry3D
{
class GeometryCollection3D;
class ISurface3D;
} // namespace Geometry3D

namespace Topology3D
{
class Topology3D;
} // namespace Topology3D

namespace Meshing
{

/// Which side of the model a point falls on, as resolved against every
/// IVolume3D in the geometry. Declared here rather than in the .cpp only so
/// that the oracle can memoize it per tetrahedron -- see
/// centroidPhaseByTetrahedron_.
enum class PointPhaseKind
{
    Exterior,
    InVolume,
    Ambiguous
};

struct PointPhase
{
    PointPhaseKind kind = PointPhaseKind::Exterior;
    std::string volumeId;
};

/**
 * @brief Decides whether a face belongs to the model boundary from its dual
 * Voronoi edge -- the textbook restricted-Delaunay test.
 *
 * A face of the tetrahedralization is dual to the segment joining the
 * circumcenters of the two tetrahedra sharing it. The restricted Delaunay
 * triangulation of a surface is the set of faces whose dual edge crosses that
 * surface, and under a dense enough sample it is homeomorphic to the surface
 * -- a provably correct boundary extracted from a volume triangulation.
 *
 * ## Known to run outside its validity conditions (OPE-186)
 *
 * The dual edge is only meaningful for well-shaped tetrahedra, and the surface
 * path never produces them: meshSurface() runs with tet-quality refinement off
 * by design, and 89.8% of tets exceed the bad-tet threshold. So the test is
 * evaluated outside its preconditions on essentially every face, permanently.
 *
 * That is what the acceptance paths in classify() below are compensating for,
 * and why each is hedged with a uniqueness requirement rather than trusted on
 * its own. It is also why enabling tet-quality refinement cuts the defect
 * count by 77% without fixing anything: a radius-edge bound provably cannot
 * eliminate slivers in 3D, and this test needs only one bad tetrahedron in the
 * wrong place.
 *
 * OPE-186 replaces this class with an oracle that decides restriction from
 * vertices and surface geometry directly, never from circumcenters. The seam
 * is deliberate: RestrictedTriangulation holds one of these by value and calls
 * only classify() and insertionPointFor(), so a replacement is a member
 * declaration and a construction site.
 */
class DualEdgeRestrictionOracle
{
public:
    DualEdgeRestrictionOracle() = default;

    /// minimumEdgeLength sizes each surface's tessellation so its cells are
    /// fine enough to classify any face whose shortest edge is at or above
    /// that floor. Built once here and never rebuilt during refinement.
    ///
    /// surfaceCandidates must outlive this object; RestrictedTriangulation
    /// owns both and keeps them together.
    DualEdgeRestrictionOracle(const Geometry3D::GeometryCollection3D& geometry,
                              const Topology3D::Topology3D& topology,
                              const SurfaceCandidates& surfaceCandidates,
                              double minimumEdgeLength);

    /// Whether face lies on the model boundary, and if so on which surface.
    /// See FaceRestriction for what each outcome means and why Unconfirmed is
    /// reported separately rather than folded into NotRestricted.
    FaceClassification classify(const FaceKey& face,
                                const MeshData3D& meshData,
                                const MeshConnectivity& connectivity,
                                const Geometry3D::GeometryCollection3D& geometry) const;

    /// Where to insert to improve the given face: the restricted Voronoi
    /// vertex, where the face's dual edge crosses the surface. Part of this
    /// class rather than of the refiner because it is a dual-edge concept --
    /// an oracle that does not use dual edges answers this question its own
    /// way.
    ///
    /// Computed on demand rather than alongside classification to avoid paying
    /// 30 bisection iterations x 3 OCC calls for every bad face when only one
    /// will actually be inserted this step. Returns nullopt if the endpoints
    /// cannot be computed or the edge does not cross the surface.
    std::optional<Point3D> insertionPointFor(const FaceKey& face,
                                             const MeshData3D& meshData,
                                             const MeshConnectivity& connectivity,
                                             const Geometry3D::ISurface3D& surface) const;

private:
    const SurfaceCandidates* surfaceCandidates_ = nullptr;

    std::vector<std::string> volumeIds_;

    // Surfaces with a seam (periodic in at least one direction -- cylinder,
    // torus, etc.). isPhaseBoundaryFace()/isUniquePhaseBoundaryCandidate()
    // are skipped for these -- see project memory: unlike ordinary
    // non-periodic creases (where this path is a validated net win), a seam
    // surface exhibits a small but persistent stream of misclassifications
    // this path can't fully arbitrate, preventing refinement from ever
    // reaching a fixed point. Root cause not yet found; this is a scoped
    // safety net, not a fix for the underlying periodic-surface interaction.
    std::unordered_set<std::string> periodicSurfaceIds_;

    // One tessellation per surface, built in the constructor at a resolution
    // derived from minimumEdgeLength and used by classify() as an exact
    // crossing oracle for this object's lifetime -- never rebuilt.
    std::unordered_map<std::string, SurfaceTessellation> surfaceTessellations_;

    /// Centroid phase memoized per tetrahedron, for the whole refinement run.
    /// `classifyPointPhase` is by far the most expensive thing this class does
    /// -- on SaddleSurfaceMesh it was 85% of total runtime, each call running
    /// BRepClass3d_SolidClassifier::Perform, which on Bezier faces drops into
    /// OCC's Extrema/global-optimisation machinery.
    ///
    /// The redundancy it removes is structural rather than accidental, and has
    /// two sources. Within one pass: a tetrahedron has four faces, and
    /// isPhaseBoundaryFace() classifies both adjacent tetrahedra of whichever
    /// face it is given, so one tetrahedron's centroid is asked for about four
    /// times. Across passes: updateAfterInsertion() reclassifies the faces of
    /// every tetrahedron incident to the new node, and each of those faces
    /// pulls in the tetrahedron on its far side -- which is an OLD, unmodified
    /// tetrahedron that was already classified in an earlier pass.
    ///
    /// Keyed by the tetrahedron's node set rather than by its element id, so
    /// it can safely outlive the elements. An element id is only a handle: it
    /// says nothing about which tetrahedron it currently names, so a cache
    /// keyed by one needs an invalidation hook on element deletion to stay
    /// honest. The node set, by contrast, is what the centroid is a function
    /// of, so an entry cannot go stale: node coordinates are fixed for the
    /// whole of refinement (RCDTMesher only moves nodes during post-refinement
    /// smoothing, after the last classification), and two distinct live
    /// tetrahedra cannot share a node set.
    mutable std::unordered_map<TetrahedronKey, PointPhase, TetrahedronKeyHash> centroidPhaseByTetrahedron_;

    /// Per-surface trimmed-boundary membership, memoized per node. Depends on
    /// the same fixed-node-coordinates invariant as
    /// centroidPhaseByTetrahedron_ above.
    mutable std::unordered_map<std::string, std::unordered_map<size_t, bool>> nodeWithinTrimmedBoundaryBySurface_;

    const PointPhase& centroidPhase(const TetrahedralElement& tetrahedron,
                                    const MeshData3D& meshData,
                                    const Geometry3D::GeometryCollection3D& geometry) const;

    bool verticesWithinTrimmedBoundary(const FaceKey& face,
                                       const std::string& surfaceId,
                                       const MeshData3D& meshData,
                                       const Geometry3D::ISurface3D& surface) const;

    bool nodeWithinTrimmedBoundary(const std::string& surfaceId,
                                   size_t nodeId,
                                   const Point3D& coordinates,
                                   const Geometry3D::ISurface3D& surface) const;

    /// Whether face is the ONLY face in the whole tetrahedralization sharing
    /// the protected edge (nodeIdA, nodeIdB) that could be restricted to
    /// surfaceId. See classify() for why the shortcut this guards needs
    /// uniqueness across the edge star rather than just a single candidate.
    bool isUniqueEdgeStarCandidate(const FaceKey& face,
                                   size_t nodeIdA,
                                   size_t nodeIdB,
                                   const std::string& surfaceId,
                                   const Geometry3D::ISurface3D& surface,
                                   const MeshData3D& meshData,
                                   const MeshConnectivity& connectivity) const;

    /// Whether the two tetrahedra sharing face fall on different sides of the
    /// model -- one inside a volume and one outside, or inside two different
    /// volumes. False when either centroid is Ambiguous, which is OPE-187's
    /// recorded root cause: an ambiguous centroid silently disables this path.
    bool isPhaseBoundaryFace(const FaceKey& face,
                             const MeshData3D& meshData,
                             const MeshConnectivity& connectivity,
                             const Geometry3D::GeometryCollection3D& geometry) const;

    /// Whether at most ONE rival face across the edge (nodeIdA, nodeIdB)
    /// could also be restricted to surfaceId by the phase test.
    ///
    /// One rival, not zero -- deliberately the opposite of
    /// isUniqueEdgeStarCandidate()'s rule above, for a reason specific to the
    /// edges each of them guards. A PROTECTED edge's two genuine incident
    /// faces belong to DIFFERENT surfaces, so for any one surfaceId only one
    /// of them is a candidate at all and a second means a spurious face. An
    /// ORDINARY edge's two genuine incident faces belong to the SAME surface,
    /// so a face's own legitimate partner is always exactly one rival:
    /// barring every rival would disqualify every honest interior face.
    /// Measured (OPE-176): the 0-rival rule gave zero improvement here.
    ///
    /// Tried on any edge of the face, not just a feature-anchored one --
    /// what justifies trusting it is isPhaseBoundaryFace()'s own robustness,
    /// not an a priori topological guarantee about the edge.
    bool isUniquePhaseBoundaryCandidate(const FaceKey& face,
                                        size_t nodeIdA,
                                        size_t nodeIdB,
                                        const std::string& surfaceId,
                                        const Geometry3D::ISurface3D& surface,
                                        const MeshData3D& meshData,
                                        const MeshConnectivity& connectivity,
                                        const Geometry3D::GeometryCollection3D& geometry) const;

    /// The two nodes of face that are chain-adjacent along one model curve, if
    /// any -- a genuinely protected edge in the Boissonnat-Oudot sense.
    static std::optional<std::pair<size_t, size_t>> findProtectedEdge(const FaceKey& face,
                                                                      const MeshData3D& meshData);

    static std::optional<size_t> findTouchedBoundingNode(size_t tetId, const MeshData3D& meshData);

    std::optional<Point3D> computeDualEdgeEndpoint(size_t tetId, const MeshData3D& meshData) const;

    /// Circumcenters of the two tetrahedra adjacent to face -- the endpoints
    /// of its dual Voronoi edge. nullopt if face has fewer than two adjacent
    /// tetrahedra, or either one's circumcenter cannot be computed.
    std::optional<std::pair<Point3D, Point3D>> computeDualEdgeEndpoints(
        const FaceKey& face,
        const MeshData3D& meshData,
        const MeshConnectivity& connectivity) const;
};

} // namespace Meshing
