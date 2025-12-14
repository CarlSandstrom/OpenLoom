#include "Edge3D.h"

namespace Topology3D
{

Edge3D::Edge3D(const std::string& id,
               const std::string& startCornerId,
               const std::string& endCornerId,
               const std::vector<std::string>& adjacentSurfaceIds) :
    id_(id),
    startCornerId_(startCornerId),
    endCornerId_(endCornerId),
    adjacentSurfaceIds_(adjacentSurfaceIds)
{
}

std::string Edge3D::getId() const
{
    return id_;
}

std::string Edge3D::getStartCornerId() const
{
    return startCornerId_;
}

std::string Edge3D::getEndCornerId() const
{
    return endCornerId_;
}

const std::vector<std::string>& Edge3D::getAdjacentSurfaceIds() const
{
    return adjacentSurfaceIds_;
}

bool Edge3D::isBoundaryEdge() const
{
    return adjacentSurfaceIds_.size() == 1;
}

bool Edge3D::isManifold() const
{
    return adjacentSurfaceIds_.size() <= 2;
}

} // namespace Topology3D