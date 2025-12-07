#include "Topology2D.h"

#include <stdexcept>

namespace Topology2D
{

Topology2D::Topology2D(const std::unordered_map<std::string, Corner2D>& corners,
                       const std::unordered_map<std::string, Edge2D>& edges,
                       const std::vector<std::string>& boundaryEdgeLoop) :
    corners_(corners),
    edges_(edges),
    boundaryEdgeLoop_(boundaryEdgeLoop)
{
}

const Corner2D& Topology2D::getCorner(const std::string& id) const
{
    auto it = corners_.find(id);
    if (it == corners_.end())
    {
        throw std::runtime_error("Corner2D not found: " + id);
    }
    return it->second;
}

const Edge2D& Topology2D::getEdge(const std::string& id) const
{
    auto it = edges_.find(id);
    if (it == edges_.end())
    {
        throw std::runtime_error("Edge2D not found: " + id);
    }
    return it->second;
}

std::vector<std::string> Topology2D::getAllCornerIds() const
{
    std::vector<std::string> ids;
    ids.reserve(corners_.size());
    for (const auto& [id, _] : corners_)
    {
        ids.push_back(id);
    }
    return ids;
}

std::vector<std::string> Topology2D::getAllEdgeIds() const
{
    std::vector<std::string> ids;
    ids.reserve(edges_.size());
    for (const auto& [id, _] : edges_)
    {
        ids.push_back(id);
    }
    return ids;
}

} // namespace Topology2D
