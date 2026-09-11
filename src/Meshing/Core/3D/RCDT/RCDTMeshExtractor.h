#pragma once

#include "Meshing/Data/3D/SurfaceMesh3D.h"
#include "Meshing/Data/3D/VolumeMesh3D.h"

#include <array>
#include <vector>

namespace Topology3D
{
class Topology3D;
} // namespace Topology3D

namespace Meshing
{

class MeshData3D;
class RestrictedTriangulation;

/// Assembles RCDTMesher's output meshes from the final refined state.
///
/// Surface triangles are the restricted faces, not every mesh face whose
/// corners lie on a CAD surface (see RestrictedTriangulation). Node IDs are
/// kept as-is, so the output node vector is indexed by mesh node ID.
class RCDTMeshExtractor
{
public:
    static SurfaceMesh3D extractSurfaceMesh(const MeshData3D& meshData,
                                            const RestrictedTriangulation& restrictedTriangulation,
                                            const Topology3D::Topology3D& topology);

    /// Emits every tetrahedron still in meshData, so the ambient tetrahedra
    /// must already have been removed.
    static VolumeMesh3D extractVolumeMesh(const MeshData3D& meshData,
                                          const RestrictedTriangulation& restrictedTriangulation,
                                          const Topology3D::Topology3D& topology);

    /// Every tetrahedron in meshData as node ID quadruplets, in the same order
    /// extractVolumeMesh() emits them.
    static std::vector<std::array<size_t, 4>> extractTetrahedra(const MeshData3D& meshData);
};

} // namespace Meshing
