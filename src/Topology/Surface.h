#pragma once

#include <string>
#include <vector>

namespace Topology {

class Surface {
public:
    Surface(const std::string& id,
            const std::vector<std::string>& boundaryEdgeIds,
            const std::vector<std::string>& cornerIds,
            const std::vector<std::string>& adjacentSurfaceIds = {},
            const std::vector<std::vector<std::string>>& edgeLoops = {});

    std::string getId() const;
    const std::vector<std::string>& getBoundaryEdgeIds() const;
    const std::vector<std::string>& getCornerIds() const;
    const std::vector<std::string>& getAdjacentSurfaceIds() const;
    const std::vector<std::vector<std::string>>& getEdgeLoops() const;
    
private:
    std::string id_;
    std::vector<std::string> boundaryEdgeIds_;      // Ordered loop(s)
    std::vector<std::string> cornerIds_;            // Vertices of surface
    std::vector<std::string> adjacentSurfaceIds_;   // Neighboring surfaces
    
    // Optional: multiple loops for holes
    std::vector<std::vector<std::string>> edgeLoops_;
};

} // namespace Topology