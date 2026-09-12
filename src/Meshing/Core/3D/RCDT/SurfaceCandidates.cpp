#include "Meshing/Core/3D/RCDT/SurfaceCandidates.h"

#include "Topology/Topology3D.h"

namespace Meshing
{

SurfaceCandidates::SurfaceCandidates(const Topology3D::Topology3D& topology)
{
    for (const auto& surfaceId : topology.getAllSurfaceIds())
        surfaceIds_.insert(surfaceId);

    for (const auto& edgeId : topology.getAllEdgeIds())
        edgeToAdjacentSurfaces_[edgeId] = topology.getEdge(edgeId).getAdjacentSurfaceIds();

    for (const auto& cornerId : topology.getAllCornerIds())
    {
        const auto& connectedSurfaces = topology.getCorner(cornerId).getConnectedSurfaceIds();
        cornerToAdjacentSurfaces_[cornerId] =
            std::vector<std::string>(connectedSurfaces.begin(), connectedSurfaces.end());
    }
}

std::unordered_set<std::string> SurfaceCandidates::effectiveSurfaceIds(
    const std::vector<std::string>& geometryIds) const
{
    std::unordered_set<std::string> result;
    for (const auto& id : geometryIds)
    {
        if (surfaceIds_.count(id))
        {
            result.insert(id);
        }
        else
        {
            const auto edgeIt = edgeToAdjacentSurfaces_.find(id);
            if (edgeIt != edgeToAdjacentSurfaces_.end())
            {
                for (const auto& surfaceId : edgeIt->second)
                    result.insert(surfaceId);
            }
            else
            {
                const auto cornerIt = cornerToAdjacentSurfaces_.find(id);
                if (cornerIt != cornerToAdjacentSurfaces_.end())
                    for (const auto& surfaceId : cornerIt->second)
                        result.insert(surfaceId);
            }
        }
    }
    return result;
}

} // namespace Meshing
