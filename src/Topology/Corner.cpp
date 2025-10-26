#include "Corner.h"

namespace Topology
{

Corner::Corner(const std::string& id,
               const std::set<std::string>& connectedEdgeIds,
               const std::set<std::string>& connectedSurfaceIds) :
    id_(id),
    connectedEdgeIds_(connectedEdgeIds),
    connectedSurfaceIds_(connectedSurfaceIds)
{
}

std::string Corner::getId() const
{
    return id_;
}

const std::set<std::string>& Corner::getConnectedEdgeIds() const
{
    return connectedEdgeIds_;
}

const std::set<std::string>& Corner::getConnectedSurfaceIds() const
{
    return connectedSurfaceIds_;
}

} // namespace Topology