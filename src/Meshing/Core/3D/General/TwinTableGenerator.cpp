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

        for (const auto& surfaceId : adjacentSurfaces)
        {
            entries.push_back({surfaceId});
        }

        table[edgeId] = std::move(entries);
    }

    return table;
}

} // namespace Meshing
