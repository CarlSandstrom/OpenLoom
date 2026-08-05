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
    /// every tetrahedral face as restricted or not.
    void buildFrom(const MeshData3D& meshData,
                   const MeshConnectivity& connectivity,
                   const Geometry3D::GeometryCollection3D& geometry,
                   const Topology3D::Topology3D& topology);

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

    /// Returns restricted faces that violate quality criteria.
    std::vector<BadRestrictedTriangle> getBadTriangles(const SurfaceMesh3DQualitySettings& settings,
                                                       const MeshData3D& meshData,
                                                       const MeshConnectivity& connectivity,
                                                       const Geometry3D::GeometryCollection3D& geometry) const;

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

    std::unordered_map<FaceKey, std::string, FaceKeyHash> restrictedFaces_;
    SurfaceProjector surfaceProjector_;

    // Built from topology in buildFrom(); reused by classifyFace() thereafter.
    std::unordered_set<std::string> surfaceIds_;
    std::unordered_map<std::string, std::vector<std::string>> edgeToAdjacentSurfaces_;
    std::unordered_map<std::string, std::vector<std::string>> cornerToAdjacentSurfaces_;

    // One tessellation per surface, built in buildFrom() and used by
    // classifyFace() as an exact crossing oracle for the lifetime of this
    // RestrictedTriangulation -- see SurfaceTessellation's class docs.
    // Mutable: classifyFace() is logically const but calls ensureResolution()
    // on this cache, which may rebuild an entry at a finer resolution.
    mutable std::unordered_map<std::string, SurfaceTessellation> surfaceTessellations_;
};

} // namespace Meshing
