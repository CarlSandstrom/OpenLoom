#include "Corner2D.h"

namespace Topology2D
{

Corner2D::Corner2D(const std::string& id, const std::set<std::string>& connectedEdgeIds) :
    id_(id),
    connectedEdgeIds_(connectedEdgeIds)
{
}

} // namespace Topology2D
