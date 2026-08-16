#pragma once

#include "Common/Types.h"
#include "Meshing/Connectivity/EdgeKey.h"
#include "Meshing/Connectivity/FaceKey.h"
#include "Meshing/Core/3D/RCDT/SurfaceProjector.h"
#include "Meshing/Core/3D/RCDT/SurfaceTessellation.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"

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

struct BadRestrictedTriangle
{
    FaceKey face;
    std::string surfaceId;
    Point3D circumcircleCenter;
    double shortestEdge;
};

/// An edge of the restricted-face set not shared by exactly 2 triangles --
/// a hole (count < 2) or a self-intersection (count > 2) in what should be a
/// closed 2-manifold. surfaceId is the surface of one restricted face
/// touching the edge (arbitrary when more than one candidate exists), used
/// as where to project a repair point onto.
struct NonManifoldRestrictedEdge
{
    EdgeKey edge;
    std::string surfaceId;
};

class RestrictedTriangulation
{
public:
    /// Full initial scan. Builds internal topology lookup tables, then classifies
    /// every tetrahedral face as restricted or not. minimumEdgeLength is used
    /// to size each surface's tessellation oracle so its cells are fine enough
    /// to correctly classify faces down to that scale — built once upfront,
    /// never rebuilt during refinement. settings is stored for the lifetime of
    /// this object (assumed constant across a refinement run) and used to
    /// evaluate each face's quality as it's (re)classified, keeping the
    /// bad-triangle set (see getBadTriangles()) incrementally maintained
    /// instead of rescanned from scratch on every query.
    void buildFrom(const MeshData3D& meshData,
                   const MeshConnectivity& connectivity,
                   const Geometry3D::GeometryCollection3D& geometry,
                   const Topology3D::Topology3D& topology,
                   double minimumEdgeLength,
                   const SurfaceMesh3DQualitySettings& settings);

    /// Incremental update after a Bowyer-Watson insertion.
    /// Removes cavity-interior faces that no longer exist, then re-classifies
    /// all faces of the new tetrahedra adjacent to newNodeId.
    void updateAfterInsertion(const std::vector<FaceKey>& cavityInteriorFaceKeys,
                              size_t newNodeId,
                              const MeshData3D& meshData,
                              const MeshConnectivity& connectivity,
                              const Geometry3D::GeometryCollection3D& geometry);

    /// Removes any restricted face whose node set contains both nodeId1 and nodeId2.
    /// Call this after splitting the curve segment between those two nodes, so that
    /// faces spanning the now-subdivided edge are not left as stale entries.
    void invalidateFacesWithEdge(size_t nodeId1, size_t nodeId2);

    /// Restricted faces that violate quality criteria, maintained incrementally
    /// (see buildFrom()/updateAfterInsertion()/invalidateFacesWithEdge()) rather
    /// than rescanned here: each face's quality is evaluated once, when it's
    /// (re)classified, not on every call to this method.
    std::vector<BadRestrictedTriangle> getBadTriangles() const;

    const std::unordered_map<FaceKey, std::string, FaceKeyHash>& getRestrictedFaces() const;

    /// The insertion point for the given bad triangle: where its dual Voronoi
    /// edge (the segment between its two adjacent tets' circumcenters) crosses
    /// the surface. Computed on demand rather than in getBadTriangles() to
    /// avoid paying 30 bisection iterations × 3 OCC calls for every bad face
    /// when only one will actually be inserted this step.
    /// Returns nullopt if the endpoints cannot be computed or the edge does not
    /// cross the surface.
    std::optional<Point3D> computeInsertionPoint(const FaceKey& face,
                                                  const MeshData3D& meshData,
                                                  const MeshConnectivity& connectivity,
                                                  const Geometry3D::ISurface3D& surface) const;

    /// Every edge of the restricted-face set whose triangle count isn't
    /// exactly 2 -- i.e. every place the set fails to be a closed 2-manifold.
    /// A watertight restricted set (what AmbientTetrahedronClassifier's flood
    /// fill requires, and what a correct RCDT run should eventually produce)
    /// returns an empty vector.
    std::vector<NonManifoldRestrictedEdge> findNonManifoldEdges() const;

private:
    /// Core restricted test for a single face.
    /// Returns the surfaceId if the face is restricted to it, nullopt otherwise.
    std::optional<std::string> classifyFace(const FaceKey& face,
                                            const MeshData3D& meshData,
                                            const MeshConnectivity& connectivity,
                                            const Geometry3D::GeometryCollection3D& geometry) const;

    /// Resolves a node's geometryIds to the set of surface IDs it touches:
    /// surface IDs pass through; edge IDs expand to adjacent surface IDs;
    /// corner IDs expand to connected surface IDs.
    std::unordered_set<std::string> effectiveSurfaceIds(const std::vector<std::string>& geometryIds) const;

    /// Whether every vertex of face lies within surface's trimmed boundary.
    /// A face whose vertices sit on the crease between two surfaces can have
    /// effectiveSurfaceIds() list both as candidates, and a dual-edge
    /// crossing test alone can't tell them apart -- the untrimmed math
    /// surfaces extend past the crease into each other's territory. Vertices
    /// are actual sample points on the real geometry, so checking them
    /// directly is robust regardless of how far a candidate's circumcenters
    /// (and thus its dual edge) happen to land from the face itself.
    bool verticesWithinTrimmedBoundary(const FaceKey& face,
                                       const MeshData3D& meshData,
                                       const Geometry3D::ISurface3D& surface) const;

    /// If two of face's three nodes are chain-adjacent along the same curve
    /// -- i.e. meshData's CurveSegmentManager has a segment directly
    /// connecting them -- returns that node pair. Property 1 of
    /// CurveProtectionScheme guarantees such a pair's protecting balls
    /// overlap, which is why classifyFace() treats the edge between them
    /// specially (see isUniqueEdgeStarCandidate()).
    static std::optional<std::pair<size_t, size_t>> findProtectedEdge(const FaceKey& face,
                                                                       const MeshData3D& meshData);

    /// Whether face is the ONLY face in the whole tetrahedralization sharing
    /// edge (nodeIdA, nodeIdB) -- its "edge star", the ring of tets
    /// surrounding that edge -- that is a genuinely LOCAL competing
    /// candidate for surfaceId: passes verticesWithinTrimmedBoundary() AND
    /// its own dual edge crosses surfaceId's tessellation. classifyFace()
    /// uses this to trust vertex classification outright on a protected edge
    /// (see findProtectedEdge()) instead of also requiring crossesSurface()
    /// on THIS face to confirm it: that oracle's near-degenerate dual-edge
    /// test is exactly what's unreliable right at a true crease. Rivals are
    /// still required to pass crossesSurface() themselves -- unlike this
    /// face, a rival several tets away isn't near-degenerate, so the oracle
    /// is reliable there, and that's what keeps the ambiguity check LOCAL:
    /// verticesWithinTrimmedBoundary alone passes for any point anywhere on
    /// surfaceId's whole CAD patch, which made an earlier version of this
    /// check (using that alone) reject almost every case as ambiguous with
    /// no effect on the final mesh (see OPE-176 project memory). Trusting
    /// ANY single-candidate face touching a protected edge, with no
    /// uniqueness check at all, was tried before that and reverted too --
    /// it accepted spurious faces from elsewhere in the edge star.
    bool isUniqueEdgeStarCandidate(const FaceKey& face,
                                   size_t nodeIdA,
                                   size_t nodeIdB,
                                   const std::string& surfaceId,
                                   const Geometry3D::ISurface3D& surface,
                                   const MeshData3D& meshData,
                                   const MeshConnectivity& connectivity) const;

    /// Whether face's two adjacent tetrahedra's CENTROIDS (not circumcenters
    /// -- see project memory: circumcenters of the severely degenerate
    /// slivers that form near creases can land arbitrarily far from the tet
    /// they claim to represent, while a centroid, being a convex combination
    /// of the tet's own vertices, never can) fall in different phases, per
    /// GeometryCollection3D's volumes (Geometry3D::IVolume3D::classifyPoint()
    /// -- an exact CAD query, not a discretized tessellation crossing test).
    /// A no-op (always false) when the geometry collection has no volumes at
    /// all, so this never changes behavior where the signal isn't available.
    bool isPhaseBoundaryFace(const FaceKey& face,
                             const MeshData3D& meshData,
                             const MeshConnectivity& connectivity,
                             const Geometry3D::GeometryCollection3D& geometry) const;

    /// Whether face is one of at most 2 candidates in edge (nodeIdA,
    /// nodeIdB)'s edge star (see isUniqueEdgeStarCandidate()) that are
    /// genuinely LOCAL competing candidates: pass verticesWithinTrimmedBoundary()
    /// AND isPhaseBoundaryFace() itself. Allowing exactly one rival, not zero,
    /// is what distinguishes this from isUniqueEdgeStarCandidate(): a
    /// protected/crease edge's two genuine incident faces belong to two
    /// DIFFERENT surfaces (a same-surface rival there is essentially never
    /// the genuine partner), but an ordinary edge's two genuine incident
    /// faces are typically for the SAME surface -- barring any rival at all
    /// makes a face's own genuine partner disqualify it, collapsing this
    /// path to a no-op (measured; see project memory). Also unlike
    /// isUniqueEdgeStarCandidate(), nodeIdA/nodeIdB can be any edge of face,
    /// not just a protected one -- what justifies trusting the result here
    /// is isPhaseBoundaryFace()'s own robustness, not an a priori
    /// topological guarantee about the edge.
    bool isUniquePhaseBoundaryCandidate(const FaceKey& face,
                                        size_t nodeIdA,
                                        size_t nodeIdB,
                                        const Geometry3D::ISurface3D& surface,
                                        const MeshData3D& meshData,
                                        const MeshConnectivity& connectivity,
                                        const Geometry3D::GeometryCollection3D& geometry) const;

    /// Which of meshData's bounding supertet nodes tetId touches, if any.
    static std::optional<size_t> findTouchedBoundingNode(size_t tetId, const MeshData3D& meshData);

    /// One endpoint of a face's dual Voronoi edge: tetId's circumcenter,
    /// unless tetId touches a bounding supertet node, in which case that
    /// node's own coordinates are used instead -- see the .cpp for why.
    /// nullopt if tetId doesn't exist or its circumcenter can't be computed.
    std::optional<Point3D> computeDualEdgeEndpoint(size_t tetId, const MeshData3D& meshData) const;

    /// Circumcenters of the two tetrahedra adjacent to face — the endpoints
    /// of its dual Voronoi edge. nullopt if face has fewer than two adjacent
    /// tetrahedra, or either one's circumcenter cannot be computed.
    std::optional<std::pair<Point3D, Point3D>> computeDualEdgeEndpoints(
        const FaceKey& face,
        const MeshData3D& meshData,
        const MeshConnectivity& connectivity) const;

    /// (Re)evaluates face's quality against settings_ and updates badFaces_
    /// accordingly -- inserted/refreshed if it fails, erased if it now
    /// passes. Called once per (re)classified face, immediately after
    /// classifyFace() confirms it's restricted to surfaceId, so badFaces_
    /// stays current without ever needing a full rescan (see getBadTriangles()).
    void updateBadFaceEntry(const FaceKey& face,
                            const std::string& surfaceId,
                            const MeshData3D& meshData,
                            const Geometry3D::GeometryCollection3D& geometry);

    std::unordered_map<FaceKey, std::string, FaceKeyHash> restrictedFaces_;
    std::unordered_map<FaceKey, BadRestrictedTriangle, FaceKeyHash> badFaces_;
    SurfaceMesh3DQualitySettings settings_;
    SurfaceProjector surfaceProjector_;

    // Built from topology in buildFrom(); reused by classifyFace() thereafter.
    std::unordered_set<std::string> surfaceIds_;
    std::unordered_map<std::string, std::vector<std::string>> edgeToAdjacentSurfaces_;
    std::unordered_map<std::string, std::vector<std::string>> cornerToAdjacentSurfaces_;
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

    // One tessellation per surface, built in buildFrom() at a resolution
    // derived from minimumEdgeLength and used by classifyFace() as an exact
    // crossing oracle for the lifetime of this RestrictedTriangulation --
    // never rebuilt during refinement.
    std::unordered_map<std::string, SurfaceTessellation> surfaceTessellations_;
};

} // namespace Meshing
