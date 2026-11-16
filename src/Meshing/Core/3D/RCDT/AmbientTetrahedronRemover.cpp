#include "Meshing/Core/3D/RCDT/AmbientTetrahedronRemover.h"

#include "Common/Exceptions/MeshException.h"
#include "Meshing/Core/3D/RCDT/AmbientTetrahedronClassifier.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Meshing/Data/3D/TetrahedronElement.h"
#include "spdlog/spdlog.h"

#include <vector>

namespace Meshing
{

void AmbientTetrahedronRemover::remove(const MeshData3D& meshData,
                                       MeshMutator3D& mutator,
                                       const RestrictedFaceMap& restrictedFaces)
{
    auto boundingNodeIds = meshData.getBoundingNodeIds();
    if (!boundingNodeIds)
        OPENLOOM_THROW_MESH(INVALID_OPERATION, "AmbientTetrahedronRemover::remove: no bounding tetrahedron in the mesh");

    std::vector<size_t> ambientTetrahedronIds;
    for (auto elementId : AmbientTetrahedronClassifier::classify(meshData, restrictedFaces))
    {
        if (dynamic_cast<const TetrahedronElement*>(meshData.getElement(elementId)))
            ambientTetrahedronIds.push_back(elementId);
    }

    for (auto tetrahedronId : ambientTetrahedronIds)
        mutator.removeElement(tetrahedronId);

    for (auto nodeId : *boundingNodeIds)
        mutator.removeNode(nodeId);
    mutator.clearBoundingNodeIds();

    spdlog::info("AmbientTetrahedronRemover::remove: Removed {} ambient tetrahedra "
                 "(true exterior + holes) and 4 bounding nodes",
                 ambientTetrahedronIds.size());
}

} // namespace Meshing
