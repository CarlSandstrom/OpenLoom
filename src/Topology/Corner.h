#pragma once

#include <string>
#include <vector>

namespace Topology {

class Corner {
public:
    Corner(const std::string& id,
           const std::vector<std::string>& connectedEdgeIds,
           const std::vector<std::string>& connectedSurfaceIds);

    std::string getId() const;
    const std::vector<std::string>& getConnectedEdgeIds() const;
    const std::vector<std::string>& getConnectedSurfaceIds() const;
    
private:
    std::string id_;
    std::vector<std::string> connectedEdgeIds_;
    std::vector<std::string> connectedSurfaceIds_;  // Explicit storage for O(1) access
};

} // namespace Topology