#include "Meshing/Core/3D/RCDT/AmbientTetrahedronRemover.h"

#include "Common/Exceptions/MeshException.h"
#include "Meshing/Core/3D/RCDT/AmbientTetrahedronClassifier.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"
#include "spdlog/spdlog.h"

#include <array>
#include <vector>

namespace Meshing
{

void AmbientTetrahedronRemover::remove(const MeshData3D& meshData,
                                       MeshMutator3D& mutator,
                                       const RestrictedTriangulation& restrictedTriangulation)
{
    if (!meshData.getBoundingNodeIds())
        OPENLOOM_THROW_MESH(INVALID_OPERATION, "AmbientTetrahedronRemover::remove: no bounding tetrahedron in the mesh");
    const std::array<size_t, 4> boundingNodeIds = *meshData.getBoundingNodeIds();

    std::vector<size_t> ambientTetrahedronIds;
    for (const size_t elementId : AmbientTetrahedronClassifier::classify(meshData, restrictedTriangulation))
    {
        if (dynamic_cast<const TetrahedralElement*>(meshData.getElement(elementId)))
            ambientTetrahedronIds.push_back(elementId);
    }

    for (const size_t tetrahedronId : ambientTetrahedronIds)
        mutator.removeElement(tetrahedronId);

    for (const size_t nodeId : boundingNodeIds)
        mutator.removeNode(nodeId);
    mutator.clearBoundingNodeIds();

    spdlog::info("AmbientTetrahedronRemover::remove: Removed {} ambient tetrahedra "
                 "(true exterior + holes) and 4 bounding nodes",
                 ambientTetrahedronIds.size());
}

} // namespace Meshing
