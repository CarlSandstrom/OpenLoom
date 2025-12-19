#pragma once

#include <array>
#include <optional>
#include <vector>

#include "Common/Types.h"
#include "Meshing/Data/MeshData3D.h"
#include "Meshing/Data/TetrahedralElement.h"
#include "Meshing/Data/TriangleElement.h"

namespace Meshing::ElementGeometry
{

template <size_t N>
bool gatherNodes(const MeshData& mesh,
                 const std::vector<size_t>& nodeIds,
                 std::array<const Node3D*, N>& nodes)
{
    if (nodeIds.size() != N)
    {
        return false;
    }

    for (size_t i = 0; i < N; ++i)
    {
        nodes[i] = mesh.getNode(nodeIds[i]);
        if (nodes[i] == nullptr)
        {
            return false;
        }
    }

    return true;
}

std::optional<double> computeQuality(const MeshData& mesh, const TetrahedralElement& element);
std::optional<double> computeQuality(const MeshData& mesh, const TriangleElement& element);

} // namespace Meshing::ElementGeometry
