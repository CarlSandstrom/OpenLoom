#pragma once

namespace Meshing
{

class MeshData3D;
class MeshMutator3D;
class RestrictedTriangulation;

/// Strips every ambient tetrahedron (see AmbientTetrahedronClassifier) from
/// the mesh, together with the bounding tetrahedron's four nodes: both the
/// seed triangulation's outer shell and, for domains with holes, the
/// tetrahedra RCDT kept triangulating interior voids with -- not just the ones
/// literally touching a bounding node.
class AmbientTetrahedronRemover
{
public:
    /// mutator must not validate node removal against a connectivity snapshot
    /// taken before refinement, since that snapshot no longer matches meshData.
    static void remove(const MeshData3D& meshData,
                       MeshMutator3D& mutator,
                       const RestrictedTriangulation& restrictedTriangulation);
};

} // namespace Meshing
