#pragma once

#include "Common/Types.h"
#include "Meshing/Connectivity/FaceKey.h"
#include "Meshing/Core/3D/RCDT/RCDTQualitySettings.h"
#include "Meshing/Core/3D/RCDT/SurfaceProjector.h"

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

    /// Where to insert if this triangle is refined: the restricted Voronoi
    /// vertex, i.e. where the face's dual Voronoi edge (the segment between
    /// its two adjacent tetrahedra's circumcenters) crosses the surface.
    /// nullopt if the face has fewer than two adjacent tetrahedra (a true
    /// boundary face of the ambient triangulation — not expected for a
    /// closed solid fully inside the bounding supertet, but possible in
    /// principle) or the crossing could not be found.
    std::optional<Point3D> insertionPoint;
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
    std::vector<BadRestrictedTriangle> getBadTriangles(const RCDTQualitySettings& settings,
                                                       const MeshData3D& meshData,
                                                       const MeshConnectivity& connectivity,
                                                       const Geometry3D::GeometryCollection3D& geometry) const;

    const std::unordered_map<FaceKey, std::string, FaceKeyHash>& getRestrictedFaces() const;

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
};

} // namespace Meshing
