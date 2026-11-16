#include "Meshing/Core/3D/RCDT/AmbientTetrahedronClassifier.h"

#include "Meshing/Connectivity/FaceKey.h"
#include "Meshing/Core/3D/RCDT/RestrictedFaceTypes.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/TetrahedronElement.h"
#include "Meshing/Data/Base/MeshConnectivity.h"

#include <queue>

namespace Meshing
{

namespace
{

constexpr ElementId INVALID_ID{SIZE_MAX};

} // namespace

std::unordered_set<ElementId>
AmbientTetrahedronClassifier::classify(const MeshData3D& meshData,
                                       const RestrictedFaceMap& restrictedFaces)
{
    std::unordered_set<ElementId> ambientTetIds;

    auto& boundingNodeIds = meshData.getBoundingNodeIds();
    if (!boundingNodeIds)
        return ambientTetIds;

    MeshConnectivity connectivity(meshData);

    std::queue<ElementId> queue;
    for (auto boundingNodeId : *boundingNodeIds)
    {
        for (auto tetId : connectivity.getNodeElements(boundingNodeId))
        {
            if (ambientTetIds.insert(tetId).second)
                queue.push(tetId);
        }
    }

    while (!queue.empty())
    {
        auto tetId = queue.front();
        queue.pop();

        auto tet = dynamic_cast<const TetrahedronElement*>(meshData.getElement(tetId));
        if (!tet)
            continue;

        for (const auto& faceArray : tet->getFaces())
        {
            FaceKey face(faceArray);
            if (restrictedFaces.count(face))
                continue; // Restricted faces are the object's boundary -- never cross them.

            const auto& [elementId0, elementId1] = connectivity.getFaceElements(face);
            auto neighborId = (elementId0 == tetId) ? elementId1 : elementId0;
            if (neighborId == INVALID_ID)
                continue;

            if (ambientTetIds.insert(neighborId).second)
                queue.push(neighborId);
        }
    }

    return ambientTetIds;
}

} // namespace Meshing
