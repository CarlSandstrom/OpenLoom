#pragma once

#include <string>
#include <vector>

namespace Topology {

class Edge {
public:
    Edge(const std::string& id,
         const std::string& startCornerId,
         const std::string& endCornerId,
         const std::vector<std::string>& adjacentSurfaceIds);

    std::string getId() const;
    std::string getStartCornerId() const;
    std::string getEndCornerId() const;
    const std::vector<std::string>& getAdjacentSurfaceIds() const;
    bool isBoundaryEdge() const;
    bool isManifold() const;
    
private:
    std::string id_;
    std::string startCornerId_;
    std::string endCornerId_;
    std::vector<std::string> adjacentSurfaceIds_;   // Usually 2, can be 1 (boundary) or >2 (non-manifold)
};

} // namespace Topology