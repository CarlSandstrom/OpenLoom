#include "Meshing/Core/3D/General/TetrahedronInversionGuard.h"

namespace Meshing
{

namespace
{

// Six times the signed volume; only its sign is used.
double orientation(const std::array<size_t, 4>& tetrahedron, const std::vector<Point3D>& nodes)
{
    auto& first = nodes[tetrahedron[0]];
    return (nodes[tetrahedron[1]] - first).cross(nodes[tetrahedron[2]] - first).dot(nodes[tetrahedron[3]] - first);
}

bool inverts(const std::array<size_t, 4>& tetrahedron,
             const std::vector<Point3D>& current,
             const std::vector<Point3D>& proposed)
{
    auto before = orientation(tetrahedron, current);
    auto after = orientation(tetrahedron, proposed);
    return (before > 0.0 && after <= 0.0) || (before < 0.0 && after >= 0.0);
}

} // namespace

TetrahedronInversionGuard::TetrahedronInversionGuard(const std::vector<std::array<size_t, 4>>& tetrahedra) :
    tetrahedra_(tetrahedra)
{
    for (size_t tetrahedronIndex = 0; tetrahedronIndex < tetrahedra_.size(); ++tetrahedronIndex)
        for (auto nodeId : tetrahedra_[tetrahedronIndex])
            tetrahedraByNode_[nodeId].push_back(tetrahedronIndex);
}

void TetrahedronInversionGuard::revertInvertingMoves(const std::vector<Point3D>& current,
                                                     std::vector<Point3D>& proposed) const
{
    std::vector<size_t> movedNodeIds;
    for (auto& [nodeId, tetrahedronIndices] : tetrahedraByNode_)
    {
        if (proposed[nodeId] != current[nodeId])
            movedNodeIds.push_back(nodeId);
    }

    bool reverted = true;
    while (reverted)
    {
        reverted = false;
        for (auto nodeId : movedNodeIds)
        {
            if (proposed[nodeId] == current[nodeId])
                continue;

            for (auto tetrahedronIndex : tetrahedraByNode_.at(nodeId))
            {
                auto& tetrahedron = tetrahedra_[tetrahedronIndex];
                if (!inverts(tetrahedron, current, proposed))
                    continue;

                for (auto cornerNodeId : tetrahedron)
                    proposed[cornerNodeId] = current[cornerNodeId];
                reverted = true;
                break;
            }
        }
    }
}

} // namespace Meshing
