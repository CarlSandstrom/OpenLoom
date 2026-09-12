#include "Meshing/Core/3D/RCDT/TetrahedronInversionGuard.h"

namespace Meshing
{

namespace
{

// Six times the signed volume; only its sign is used.
double orientation(const std::array<size_t, 4>& tetrahedron, const std::vector<Point3D>& nodes)
{
    const Point3D& first = nodes[tetrahedron[0]];
    return (nodes[tetrahedron[1]] - first).cross(nodes[tetrahedron[2]] - first).dot(nodes[tetrahedron[3]] - first);
}

bool inverts(const std::array<size_t, 4>& tetrahedron,
             const std::vector<Point3D>& current,
             const std::vector<Point3D>& proposed)
{
    const double before = orientation(tetrahedron, current);
    const double after = orientation(tetrahedron, proposed);
    return (before > 0.0 && after <= 0.0) || (before < 0.0 && after >= 0.0);
}

} // namespace

TetrahedronInversionGuard::TetrahedronInversionGuard(const std::vector<std::array<size_t, 4>>& tetrahedra) :
    tetrahedra_(tetrahedra)
{
    for (size_t tetrahedronIndex = 0; tetrahedronIndex < tetrahedra_.size(); ++tetrahedronIndex)
        for (const size_t nodeId : tetrahedra_[tetrahedronIndex])
            tetrahedraByNode_[nodeId].push_back(tetrahedronIndex);
}

void TetrahedronInversionGuard::revertInvertingMoves(const std::vector<Point3D>& current,
                                                     std::vector<Point3D>& proposed) const
{
    std::vector<size_t> movedNodeIds;
    for (const auto& [nodeId, tetrahedronIndices] : tetrahedraByNode_)
    {
        if (proposed[nodeId] != current[nodeId])
            movedNodeIds.push_back(nodeId);
    }

    bool reverted = true;
    while (reverted)
    {
        reverted = false;
        for (const size_t nodeId : movedNodeIds)
        {
            if (proposed[nodeId] == current[nodeId])
                continue;

            for (const size_t tetrahedronIndex : tetrahedraByNode_.at(nodeId))
            {
                const auto& tetrahedron = tetrahedra_[tetrahedronIndex];
                if (!inverts(tetrahedron, current, proposed))
                    continue;

                for (const size_t cornerNodeId : tetrahedron)
                    proposed[cornerNodeId] = current[cornerNodeId];
                reverted = true;
                break;
            }
        }
    }
}

} // namespace Meshing
