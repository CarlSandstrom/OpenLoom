#include "Meshing/Core/3D/RCDT/RegularConflictRegion.h"

#include "Meshing/Connectivity/FaceKey.h"
#include "Meshing/Core/3D/General/RegularPredicates3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"
#include "Meshing/Data/Base/MeshConnectivity.h"

#include <cstdint>
#include <deque>
#include <unordered_set>

namespace Meshing::RegularConflictRegion
{

namespace
{

constexpr std::size_t INVALID_ID = SIZE_MAX;

bool conflicts(const TetrahedralElement& tetrahedron, const Point3D& point, const MeshData3D& meshData)
{
    const auto& ids = tetrahedron.getNodeIds();
    const Node3D* n0 = meshData.getNode(ids[0]);
    const Node3D* n1 = meshData.getNode(ids[1]);
    const Node3D* n2 = meshData.getNode(ids[2]);
    const Node3D* n3 = meshData.getNode(ids[3]);
    return RegularPredicates3D::insidePointOrthosphere(n0->getCoordinates(), n0->getWeight(), n1->getCoordinates(),
                                                       n1->getWeight(), n2->getCoordinates(), n2->getWeight(),
                                                       n3->getCoordinates(), n3->getWeight(), point, 0.0);
}

} // namespace

std::vector<std::size_t> find(const std::array<std::size_t, 2>& seeds,
                              const Point3D& point,
                              const MeshData3D& meshData,
                              const MeshConnectivity& connectivity)
{
    std::vector<std::size_t> region;
    std::unordered_set<std::size_t> visited;
    std::deque<std::size_t> pending;
    for (const std::size_t seed : seeds)
        if (seed != INVALID_ID && visited.insert(seed).second)
            pending.push_back(seed);

    while (!pending.empty())
    {
        const std::size_t elementId = pending.front();
        pending.pop_front();
        const auto* tetrahedron = dynamic_cast<const TetrahedralElement*>(meshData.getElement(elementId));
        if (!tetrahedron || !conflicts(*tetrahedron, point, meshData))
            continue;
        region.push_back(elementId);
        for (const auto& faceNodes : tetrahedron->getFaces())
        {
            const auto& [first, second] = connectivity.getFaceElements(FaceKey(faceNodes));
            for (const std::size_t neighbour : {first, second})
                if (neighbour != INVALID_ID && visited.insert(neighbour).second)
                    pending.push_back(neighbour);
        }
    }
    return region;
}

bool isHiddenOrDuplicate(const Point3D& point, const std::vector<std::size_t>& region, const MeshData3D& meshData)
{
    for (const std::size_t elementId : region)
    {
        const auto* tetrahedron = dynamic_cast<const TetrahedralElement*>(meshData.getElement(elementId));
        for (const std::size_t nodeId : tetrahedron->getNodeIds())
        {
            const Node3D* node = meshData.getNode(nodeId);
            if ((node->getCoordinates() - point).squaredNorm() <= node->getWeight())
                return true;
        }
    }
    return false;
}

} // namespace Meshing::RegularConflictRegion
