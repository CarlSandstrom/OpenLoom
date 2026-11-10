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
bool conflicts(const TetrahedralElement& tetrahedron, const Point3D& point, const MeshData3D& meshData);
} // namespace

std::vector<std::size_t> find(const std::array<std::size_t, 2>& seeds,
                              const Point3D& point,
                              const MeshData3D& meshData,
                              const MeshConnectivity& connectivity)
{
    std::vector<std::size_t> region;
    std::unordered_set<std::size_t> visited;
    std::deque<std::size_t> pending;
    for (auto seed : seeds)
        if (seed != INVALID_ID && visited.insert(seed).second)
            pending.push_back(seed);

    while (!pending.empty())
    {
        auto elementId = pending.front();
        pending.pop_front();
        auto tetrahedron = dynamic_cast<const TetrahedralElement*>(meshData.getElement(elementId));
        if (!tetrahedron || !conflicts(*tetrahedron, point, meshData))
            continue;
        region.push_back(elementId);
        for (const auto& faceNodes : tetrahedron->getFaces())
        {
            const auto& [first, second] = connectivity.getFaceElements(FaceKey(faceNodes));
            for (auto neighbour : {first, second})
                if (neighbour != INVALID_ID && visited.insert(neighbour).second)
                    pending.push_back(neighbour);
        }
    }
    return region;
}

bool isHiddenOrDuplicate(const Point3D& point, const std::vector<std::size_t>& region, const MeshData3D& meshData)
{
    for (auto elementId : region)
    {
        auto tetrahedron = dynamic_cast<const TetrahedralElement*>(meshData.getElement(elementId));
        for (auto nodeId : tetrahedron->getNodeIds())
        {
            auto node = meshData.getNode(nodeId);
            if ((node->getCoordinates() - point).squaredNorm() <= node->getWeight())
                return true;
        }
    }
    return false;
}

namespace
{

bool conflicts(const TetrahedralElement& tetrahedron, const Point3D& point, const MeshData3D& meshData)
{
    auto& nodeIds = tetrahedron.getNodeIds();
    auto node0 = meshData.getNode(nodeIds[0]);
    auto node1 = meshData.getNode(nodeIds[1]);
    auto node2 = meshData.getNode(nodeIds[2]);
    auto node3 = meshData.getNode(nodeIds[3]);
    return RegularPredicates3D::insidePointOrthosphere(
        node0->getCoordinates(), node0->getWeight(), node1->getCoordinates(), node1->getWeight(),
        node2->getCoordinates(), node2->getWeight(), node3->getCoordinates(), node3->getWeight(),
        point, 0.0);
}

} // namespace

} // namespace Meshing::RegularConflictRegion
