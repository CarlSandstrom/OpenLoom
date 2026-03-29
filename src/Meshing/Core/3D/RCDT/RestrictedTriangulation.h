#pragma once

#include "Common/Types.h"
#include "Meshing/Connectivity/FaceKey.h"
#include "Meshing/Core/3D/RCDT/RCDTQualitySettings.h"
#include "Meshing/Core/3D/RCDT/SurfaceProjector.h"

#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
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

    /// Returns restricted faces that violate quality criteria.
    std::vector<BadRestrictedTriangle> getBadTriangles(const RCDTQualitySettings& settings,
                                                       const MeshData3D& meshData,
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
    /// direct surface IDs pass through; edge IDs expand to adjacent surface IDs.
    std::unordered_set<std::string> effectiveSurfaceIds(const std::vector<std::string>& geometryIds) const;

    std::unordered_map<FaceKey, std::string, FaceKeyHash> restrictedFaces_;
    SurfaceProjector surfaceProjector_;

    // Built from topology in buildFrom(); reused by classifyFace() thereafter.
    std::unordered_set<std::string> surfaceIds_;
    std::unordered_map<std::string, std::vector<std::string>> edgeToAdjacentSurfaces_;
};

} // namespace Meshing
