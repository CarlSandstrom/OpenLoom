#include "Surface3D.h"

namespace Topology3D
{

Surface3D::Surface3D(const std::string& id,
                     const std::vector<std::string>& boundaryEdgeIds,
                     const std::vector<std::string>& cornerIds,
                     const std::vector<std::string>& adjacentSurfaceIds,
                     const std::vector<std::vector<std::string>>& edgeLoops) :
    id_(id),
    boundaryEdgeIds_(boundaryEdgeIds),
    cornerIds_(cornerIds),
    adjacentSurfaceIds_(adjacentSurfaceIds),
    edgeLoops_(edgeLoops)
{
}

std::string Surface3D::getId() const
{
    return id_;
}

const std::vector<std::string>& Surface3D::getBoundaryEdgeIds() const
{
    return boundaryEdgeIds_;
}

const std::vector<std::string>& Surface3D::getCornerIds() const
{
    return cornerIds_;
}

const std::vector<std::string>& Surface3D::getAdjacentSurfaceIds() const
{
    return adjacentSurfaceIds_;
}

const std::vector<std::vector<std::string>>& Surface3D::getEdgeLoops() const
{
    return edgeLoops_;
}

} // namespace Topology3D