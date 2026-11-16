#pragma once

#include "Meshing/Core/3D/RCDT/RestrictedFaceTypes.h"

namespace Meshing
{

class MeshingContext3D;

/// Strips every ambient tetrahedron (see AmbientTetrahedronClassifier) from
/// the mesh, together with the bounding tetrahedron's four nodes: both the
/// seed triangulation's outer shell and, for domains with holes, the
/// tetrahedra RCDT kept triangulating interior voids with -- not just the ones
/// literally touching a bounding node.
class AmbientTetrahedronRemover
{
public:
    static void remove(MeshingContext3D& context, const RestrictedFaceMap& restrictedFaces);
};

} // namespace Meshing
