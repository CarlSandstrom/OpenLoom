#pragma once

#include "Common/Types.h"
#include "Meshing/Connectivity/EdgeKey.h"
#include "Meshing/Connectivity/FaceKey.h"
#include "Meshing/Connectivity/TetrahedronKey.h"
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
class CurveSegmentManager;
class MeshConnectivity;
class MeshData3D;
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

struct BadRestrictedTriangle
{
    FaceKey face;
    std::string surfaceId;
    Point3D circumcircleCenter;
    double shortestEdge;
};

/// How an edge's restricted-face coverage departs from what the CAD
/// topology calls for -- see findNonManifoldEdges() for the invariant itself.
enum class RestrictedEdgeDefect
{
    /// Fewer incident faces than expected: a hole in the surface.
    MissingFace,
    /// More incident faces than expected: the same piece of surface covered
    /// twice, the over-acceptance flap of OPE-184.
    ExcessFace,
    /// The expected NUMBER of faces, but restricted to the wrong surfaces --
    /// e.g. two faces on an edge that lies in a surface's interior but that
    /// disagree about which surface, or a curve whose two incident faces
    /// both claim the same one of its two adjacent surfaces.
    SurfaceMismatch
};

/// An edge whose incident restricted faces do not match what the CAD
/// topology calls for. surfaceId is where a repair point should be projected:
/// the surface the edge is short of a face on when one is missing, otherwise
/// the surface carrying the excess.
struct NonManifoldRestrictedEdge
{
    EdgeKey edge;
    std::string surfaceId;
    RestrictedEdgeDefect defect = RestrictedEdgeDefect::MissingFace;
};

/// What RestrictedTriangulation::removeDefectiveFaces() removed, and the
/// non-manifold edges it could not resolve, counted per RestrictedEdgeDefect.
struct DefectiveFaceRemovalSummary
{
    size_t chordFacesRemoved = 0;
    size_t excessFacesRemoved = 0;
    size_t remainingMissingFaceEdges = 0;
    size_t remainingExcessFaceEdges = 0;
    size_t remainingSurfaceMismatchEdges = 0;
};

/// Which side of the model a point falls on, as resolved against every
/// IVolume3D in the geometry. Lives here rather than in the .cpp only so
/// that RestrictedTriangulation can memoize it per tetrahedron -- see
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

    /// Post-hoc cleanup, meant to be called ONCE after refinement has fully
    /// converged: removeChordFaces(), then removeExcessFaces() -- in that
    /// order, since the chord removals change the per-edge counts
    /// removeExcessFaces() judges -- then counts the non-manifold edges
    /// neither could resolve (see findNonManifoldEdges()). Counted per defect
    /// kind because the three want different fixes -- a hole is a face
    /// classification never made, an excess is one made twice -- and a single
    /// total cannot tell them apart, which has misled this area before.
    DefectiveFaceRemovalSummary removeDefectiveFaces(const MeshData3D& meshData);

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

    /// Every edge whose incident restricted faces do not match what the CAD
    /// topology calls for there. A restricted set that satisfies the
    /// invariant everywhere (what AmbientTetrahedronClassifier's flood fill
    /// requires, and what a correct RCDT run should eventually produce)
    /// returns an empty vector.
    ///
    /// The invariant is per-edge and depends on whether the edge lies ON a
    /// model curve -- i.e. its two nodes are chain-adjacent along one, per
    /// meshData's CurveSegmentManager:
    ///
    ///  * On a curve: exactly one incident face per surface adjacent to that
    ///    curve, as listed by Topology3D::Edge3D::getAdjacentSurfaceIds().
    ///    Two for an ordinary crease, one for a free boundary, and THREE OR
    ///    MORE at a junction where that many surfaces meet.
    ///  * Not on a curve (a surface interior, or a chord skipping a curve's
    ///    own sample points): exactly 2 incident faces, both restricted to
    ///    the same surface.
    ///
    /// This is deliberately NOT the flat "every edge has exactly 2 faces"
    /// test it replaces. That test states a closed-2-manifold requirement
    /// the models this library targets do not all satisfy: in a conformal
    /// multi-material model -- a polycrystal or multiphase microstructure --
    /// grain boundaries meet along TRIPLE LINES where three boundary patches
    /// share one edge, and at quadruple points where four triple lines meet.
    /// Three faces on such an edge is the equilibrium configuration, not a
    /// defect, and Edge3D has always documented its adjacency list as
    /// "usually 2, can be 1 (boundary) or >2 (non-manifold)". Reading the
    /// expected count off the topology rather than assuming 2 is what lets
    /// a legitimate junction and an over-acceptance flap be told apart.
    ///
    /// The distinction also matters for the flap itself: the count test
    /// lumps holes, duplicates and (in future) junctions into one number, so
    /// it cannot serve as a quality gate, and any repair of the "keep the
    /// best 2, drop the rest" shape built on it would silently destroy
    /// triple lines. See OPE-184.
    std::vector<NonManifoldRestrictedEdge> findNonManifoldEdges(const MeshData3D& meshData) const;

private:
    /// Removes every currently-restricted face that uses a same-curve chord
    /// edge (see hasSameCurveChordEdge()) -- an edge between two points on
    /// the same curve that aren't chain-adjacent, skipping over the curve's
    /// own intermediate sample points. Meant to be called ONCE, after
    /// refinement has fully converged. Rejecting chord faces during
    /// classification itself was tried first and measured to regress
    /// SaddleSurfaceMesh badly (28->69 non-manifold edges): it denies
    /// refinement's normal self-correcting process the chance to naturally
    /// supersede most chord faces with the correct fine chain before this
    /// runs (measured: buildInitial() alone had ~24 more chord faces than
    /// the converged mesh's residual 15, most resolved on their own by
    /// refinement's end). A post-hoc cleanup instead finds the correct
    /// alternative already in place for that much smaller residual set --
    /// see OPE-176 project memory for the full investigation.
    ///
    /// A chord face is only dropped while its NON-chord edges can spare the
    /// triangle -- i.e. none of them is at exactly 2. A face has 3 edges but
    /// only the chord one is the problem; removing the face over that one
    /// edge also takes the other two down with it, and an unconditional
    /// version of this (measured on SaddleSurfaceMesh) tore fresh holes in
    /// otherwise-healthy surface elsewhere, trading duplicates here for gaps
    /// there. Applied greedily to a fixed point, since each removal changes
    /// the counts the remaining candidates are judged against.
    ///
    /// Returns the number of faces removed.
    size_t removeChordFaces(const MeshData3D& meshData);

    /// Post-hoc manifold enforcement: removes flaps of restricted faces that
    /// cover a piece of surface already covered, leaving the edges they were
    /// piled onto with exactly the number of faces the CAD topology calls for
    /// (see findNonManifoldEdges() for that invariant). Meant to be called
    /// ONCE, after refinement has fully converged and after
    /// removeChordFaces(), whose removals change the counts this judges.
    ///
    /// A flap is identified as a whole CONNECTED COMPONENT rather than
    /// face-by-face. Faces are joined through every edge that is NOT
    /// over-covered, so an over-covered edge acts as a cut: a flap laid over
    /// an otherwise correct sheet meets that sheet only along over-covered
    /// edges -- its own interior edges carry just its own two triangles --
    /// and so falls out as a component of its own. This is what OPE-184
    /// measured the defect to be: not 222 independent faults but 21 doubled
    /// patches of 4-7 elements across, the seam between patch and sheet
    /// showing up as the multiplicity-3 edges.
    ///
    /// A component is removed only when it touches an over-covered edge at
    /// all, and only while every edge it touches can spare its faces: what
    /// remains after the removal must still meet the expected count, or must
    /// be nothing at all (a flap's own interior edges leave the restricted
    /// set along with it, so they cannot be left as holes). The largest
    /// component is never removed. Applied greedily to a fixed point, since
    /// each removal changes the counts the remaining candidates are judged
    /// against.
    ///
    /// Deliberately NOT "for any edge with more than 2 faces, keep the best
    /// 2 and drop the rest": the expected count is read off the topology,
    /// so an edge where three surfaces genuinely meet -- a triple line in a
    /// multi-material model -- expects 3 and is never touched here.
    ///
    /// Returns the number of faces removed.
    size_t removeExcessFaces(const MeshData3D& meshData);

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

    const PointPhase& centroidPhase(const TetrahedralElement& tetrahedron,
                                    const MeshData3D& meshData,
                                    const Geometry3D::GeometryCollection3D& geometry) const;

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
                                       const std::string& surfaceId,
                                       const MeshData3D& meshData,
                                       const Geometry3D::ISurface3D& surface) const;

    /// Whether a single node lies within surface's trimmed boundary, memoized
    /// per (surface, node) for the whole refinement run.
    ///
    /// The projection this performs is the single most expensive thing in
    /// classifyFace(): ShapeAnalysis_Surface::ValueOfUV runs a global
    /// Extrema_GenExtPS search whose grid is rebuilt per call on a Bezier or
    /// NURBS surface, and caching the ShapeAnalysis_Surface analyzer (as
    /// OpenCascadeSurface already does) does not avoid that. Measured at 49%
    /// of SaddleSurfaceMesh's total instructions.
    ///
    /// The redundancy is structural: a node is a vertex of many faces, every
    /// one of which projects all three of its vertices, and those faces are
    /// reclassified repeatedly as refinement proceeds around them. The answer
    /// depends only on the node's coordinates and the surface, both of which
    /// are fixed for the whole of refinement -- the same invariant that lets
    /// centroidPhaseByTetrahedron_ outlive a single pass.
    ///
    /// Takes surfaceId rather than calling surface.getId(): the OpenCascade
    /// implementation of getId() formats a string through an ostringstream on
    /// every call, which would cost more than the lookup it keys.
    bool nodeWithinTrimmedBoundary(const std::string& surfaceId,
                                   size_t nodeId,
                                   const Point3D& coordinates,
                                   const Geometry3D::ISurface3D& surface) const;

    mutable std::unordered_map<std::string, std::unordered_map<size_t, bool>> nodeWithinTrimmedBoundaryBySurface_;

    /// One edge's restricted-face coverage set against what the CAD topology
    /// calls for there -- the raw material of the invariant documented on
    /// findNonManifoldEdges(), shared by that check and by
    /// removeExcessFaces().
    struct RestrictedEdgeCoverage
    {
        std::vector<FaceKey> incidentFaces;
        std::unordered_map<std::string, size_t> actualBySurface;

        /// How many faces the edge should carry in total. Always populated.
        size_t expectedCount = 0;

        /// Which surfaces those faces should belong to. Empty when the edge
        /// lies off any model curve AND its faces already disagree about the
        /// surface: the count is still expected to be 2, but the faces
        /// themselves pin down no surface to expect them on.
        std::unordered_map<std::string, size_t> expectedBySurface;
    };

    using RestrictedEdgeCoverageMap = std::unordered_map<EdgeKey, RestrictedEdgeCoverage, EdgeKeyHash>;

    /// Every edge of the restricted-face set, with its incident faces and the
    /// coverage the CAD topology expects of it.
    RestrictedEdgeCoverageMap buildEdgeCoverage(const MeshData3D& meshData) const;

    /// How many incident restricted faces each surface should carry on edge,
    /// per the CAD topology -- the expectation findNonManifoldEdges() checks
    /// the actual coverage against. Returns nullopt when edge does not lie on
    /// a model curve, where the topology fixes no expectation of its own and
    /// the surface-interior rule applies instead.
    ///
    /// An edge lies on a model curve when its two nodes are chain-adjacent
    /// along one. Sharing a curve is NOT enough: two non-adjacent sample
    /// points of the same curve are joined by a chord that skips the points
    /// between them (see hasSameCurveChordEdge()), which is an ordinary edge
    /// of the ambient tetrahedralization rather than a piece of the crease,
    /// and inherits no expectation from it.
    std::optional<std::unordered_map<std::string, size_t>> expectedIncidentSurfaces(
        const EdgeKey& edge,
        const CurveSegmentManager& curveSegmentManager) const;

    /// If two of face's three nodes are chain-adjacent along the same curve
    /// -- i.e. meshData's CurveSegmentManager has a segment directly
    /// connecting them -- returns that node pair. Property 1 of
    /// CurveProtectionScheme guarantees such a pair's protecting balls
    /// overlap, which is why classifyFace() treats the edge between them
    /// specially (see isUniqueEdgeStarCandidate()).
    static std::optional<std::pair<size_t, size_t>> findProtectedEdge(const FaceKey& face,
                                                                      const MeshData3D& meshData);

    /// Whether face has an edge between two points on the SAME curve that
    /// aren't chain-adjacent (a "chord" skipping over the curve's own
    /// intermediate sample points, as opposed to a genuine protected edge --
    /// see findProtectedEdge()). Such a face can never be a genuine
    /// restricted triangle: the true crease follows the curved path through
    /// the skipped points, not a straight shortcut past them. This is
    /// normal, unavoidable Delaunay behavior, not a defect in the
    /// tetrahedralization itself -- Delaunay only knows where points sit in
    /// 3D, not that several of them are meant to lie between two others
    /// along a curved path, so a direct edge between non-adjacent curve
    /// points is a perfectly ordinary edge of the ambient tetrahedralization
    /// (the same way a Delaunay triangulation of points on a circle has
    /// internal diagonals alongside the boundary polygon). The defect is
    /// entirely in trusting a FACE built on that edge as if it were part of
    /// the boundary -- see removeChordFaces() for how this is acted on (a
    /// post-hoc cleanup, not a classification-time rejection -- see its own
    /// doc for why).
    bool hasSameCurveChordEdge(const FaceKey& face, const MeshData3D& meshData) const;

    /// Whether the single edge (nodeIdA, nodeIdB) is such a chord -- the
    /// per-edge form of hasSameCurveChordEdge(). removeChordFaces() needs to
    /// tell a face's chord edge apart from its other two, so it only weighs
    /// the collateral cost of the edges the removal isn't meant to fix.
    bool isSameCurveChordEdge(size_t nodeIdA, size_t nodeIdB, const MeshData3D& meshData) const;

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
                                        const std::string& surfaceId,
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
