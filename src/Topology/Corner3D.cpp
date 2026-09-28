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

} // namespace Topology3D