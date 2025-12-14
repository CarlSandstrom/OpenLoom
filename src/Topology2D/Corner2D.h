#pragma once

#include <set>
#include <string>

namespace Topology2D
{

/**
 * @brief Topological corner (vertex) in 2D
 *
 * Represents a vertex in the 2D domain with connectivity information.
 */
class Corner2D
{
public:
    Corner2D(const std::string& id, const std::set<std::string>& connectedEdgeIds);

    std::string getId() const { return id_; }
    const std::set<std::string>& getConnectedEdgeIds() const { return connectedEdgeIds_; }

private:
    std::string id_;
    std::set<std::string> connectedEdgeIds_;
};

} // namespace Topology2D
