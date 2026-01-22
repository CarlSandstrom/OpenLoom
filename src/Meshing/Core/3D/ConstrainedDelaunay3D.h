#pragma once

#include <array>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "Common/Types.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Meshing/Core/2D/ConstrainedDelaunay2D.h"
#include "Meshing/Core/2D/MeshingContext2D.h"
#include "Meshing/Core/3D/MeshingContext3D.h"
#include "Meshing/Core/ConstraintStructures.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"
#include "Topology/Topology3D.h"

namespace Meshing
{

class ConstrainedDelaunay3D
{
public:
    explicit ConstrainedDelaunay3D(MeshingContext3D& context);

    /// Get a tetrahedron by ID (returns nullptr if not found or not a tetrahedron)
    const TetrahedralElement* getTetrahedralElement(size_t id) const;

private:
    MeshingContext3D& context_;
    MeshData3D& meshData_;
    MeshMutator3D& meshMutator_;
};

} // namespace Meshing
