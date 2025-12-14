#include "Edge2D.h"

namespace Topology2D
{

Edge2D::Edge2D(const std::string& id,
               const std::string& startCornerId,
               const std::string& endCornerId) :
    id_(id),
    startCornerId_(startCornerId),
    endCornerId_(endCornerId)
{
}

} // namespace Topology2D
