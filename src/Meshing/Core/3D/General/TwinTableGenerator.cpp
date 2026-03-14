#include "Meshing/Core/3D/General/TwinTableGenerator.h"
#include "Topology/Edge3D.h"
#include "Topology/Topology3D.h"

namespace Meshing
{

EdgeTwinTable TwinTableGenerator::generate(const Topology3D::Topology3D& topology)
{
    EdgeTwinTable table;

    for (const auto& edgeId : topology.getAllEdgeIds())
    {
        const auto& edge = topology.getEdge(edgeId);
        const auto& adjacentSurfaces = edge.getAdjacentSurfaceIds();

        // Boundary edges (single adjacent surface) have no twins
        if (adjacentSurfaces.size() < 2)
        {
            continue;
        }

        std::vector<EdgeTwinEntry> entries;
        entries.reserve(adjacentSurfaces.size());

        for (size_t i = 0; i < adjacentSurfaces.size(); ++i)
        {
            TwinOrientation orientation = (i == 0) ? TwinOrientation::Same : TwinOrientation::Reversed;
            entries.push_back({adjacentSurfaces[i], orientation});
        }

        table[edgeId] = std::move(entries);
    }

    return table;
}

} // namespace Meshing
