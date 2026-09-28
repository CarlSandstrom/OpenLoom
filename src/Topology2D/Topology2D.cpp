#include "Topology2D.h"

#include "Common/Exceptions/TopologyException.h"

namespace Topology2D
{

Topology2D::Topology2D(const std::unordered_map<std::string, Corner2D>& corners,
                       const std::unordered_map<std::string, Edge2D>& edges,
                       const std::vector<std::string>& boundaryEdgeLoop) :
    corners_(corners),
    edges_(edges),
    outerEdgeLoop_(boundaryEdgeLoop)
{
}

Topology2D::Topology2D(const std::unordered_map<std::string, Corner2D>& corners,
                       const std::unordered_map<std::string, Edge2D>& edges,
                       const std::vector<std::string>& outerEdgeLoop,
                       const std::vector<std::vector<std::string>>& holeEdgeLoops) :
    corners_(corners),
    edges_(edges),
    outerEdgeLoop_(outerEdgeLoop),
    holeEdgeLoops_(holeEdgeLoops)
{
}

const Edge2D& Topology2D::getEdge(const std::string& id) const
{
    auto it = edges_.find(id);
    if (it == edges_.end())
    {
        OPENLOOM_THROW_TOPOLOGY(ENTITY_NOT_FOUND, "Edge2D not found: " + id);
    }
    return it->second;
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
