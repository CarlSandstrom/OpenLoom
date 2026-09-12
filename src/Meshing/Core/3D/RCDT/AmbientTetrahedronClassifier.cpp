#include "Meshing/Core/3D/RCDT/AmbientTetrahedronClassifier.h"

#include "Meshing/Connectivity/FaceKey.h"
#include "Meshing/Core/3D/RCDT/RestrictedTriangulation.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"
#include "Meshing/Data/Base/MeshConnectivity.h"

#include <queue>

namespace Meshing
{

namespace
{

constexpr size_t INVALID_ID = SIZE_MAX;

} // namespace

std::unordered_set<size_t>
AmbientTetrahedronClassifier::classify(const MeshData3D& meshData,
                                       const RestrictedTriangulation& restrictedTriangulation)
{
    std::unordered_set<size_t> ambientTetIds;

    const auto& boundingNodeIds = meshData.getBoundingNodeIds();
    if (!boundingNodeIds)
        return ambientTetIds;

    const MeshConnectivity connectivity(meshData);
    const auto& restrictedFaces = restrictedTriangulation.getRestrictedFaces();

    std::queue<size_t> queue;
    for (const size_t boundingNodeId : *boundingNodeIds)
    {
        for (const size_t tetId : connectivity.getNodeElements(boundingNodeId))
        {
            if (ambientTetIds.insert(tetId).second)
                queue.push(tetId);
        }
    }

    while (!queue.empty())
    {
        const size_t tetId = queue.front();
        queue.pop();

        const auto* tet = dynamic_cast<const TetrahedralElement*>(meshData.getElement(tetId));
        if (!tet)
            continue;

        for (const auto& faceArray : tet->getFaces())
        {
            const FaceKey face(faceArray);
            if (restrictedFaces.count(face))
                continue; // Restricted faces are the object's boundary -- never cross them.

            const auto& [elementId1, elementId2] = connectivity.getFaceElements(face);
            const size_t neighborId = (elementId1 == tetId) ? elementId2 : elementId1;
            if (neighborId == INVALID_ID)
                continue;

            if (ambientTetIds.insert(neighborId).second)
                queue.push(neighborId);
        }
    }

    return ambientTetIds;
}

} // namespace Meshing
