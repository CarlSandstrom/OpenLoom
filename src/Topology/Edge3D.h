#pragma once

#include <string>
#include <vector>

namespace Topology3D
{

class Edge3D
{
public:
    Edge3D(const std::string& id,
           const std::string& startCornerId,
           const std::string& endCornerId,
           const std::vector<std::string>& adjacentSurfaceIds);

    std::string getStartCornerId() const;
    std::string getEndCornerId() const;
    const std::vector<std::string>& getAdjacentSurfaceIds() const;

private:
    std::string id_;
    std::string startCornerId_;
    std::string endCornerId_;
    std::vector<std::string> adjacentSurfaceIds_; // Usually 2, can be 1 (boundary) or >2 (non-manifold)
};

} // namespace Topology3D
