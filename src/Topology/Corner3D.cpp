#include "Corner3D.h"

namespace Topology3D
{

Corner3D::Corner3D(const std::string& id,
                   const std::set<std::string>& connectedEdgeIds,
                   const std::set<std::string>& connectedSurfaceIds) :
    id_(id),
    connectedEdgeIds_(connectedEdgeIds),
    connectedSurfaceIds_(connectedSurfaceIds)
{
}

std::string Corner3D::getId() const
{
    return id_;
}

const std::set<std::string>& Corner3D::getConnectedEdgeIds() const
{
    return connectedEdgeIds_;
}

const std::set<std::string>& Corner3D::getConnectedSurfaceIds() const
{
    return connectedSurfaceIds_;
}

} // namespace Topology3D