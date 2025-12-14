#pragma once

#include <set>
#include <string>

namespace Topology3D
{

class Corner3D
{
public:
    Corner3D(const std::string& id,
             const std::set<std::string>& connectedEdgeIds,
             const std::set<std::string>& connectedSurfaceIds);

    std::string getId() const;
    const std::set<std::string>& getConnectedEdgeIds() const;
    const std::set<std::string>& getConnectedSurfaceIds() const;

private:
    std::string id_;
    std::set<std::string> connectedEdgeIds_;
    std::set<std::string> connectedSurfaceIds_;
};

} // namespace Topology3D