#include "Topology2D/Edge2D.h"

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

const std::string& Edge2D::getId() const
{
    return id_;
}

const std::string& Edge2D::getStartCornerId() const
{
    return startCornerId_;
}

const std::string& Edge2D::getEndCornerId() const
{
    return endCornerId_;
}

} // namespace Topology2D
