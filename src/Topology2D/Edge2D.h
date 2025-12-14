#pragma once

#include <string>

namespace Topology2D
{

/**
 * @brief Topological edge in 2D
 *
 * Represents a boundary edge connecting two corners in the 2D domain.
 */
class Edge2D
{
public:
    Edge2D(const std::string& id,
           const std::string& startCornerId,
           const std::string& endCornerId);

    std::string getId() const { return id_; }
    std::string getStartCornerId() const { return startCornerId_; }
    std::string getEndCornerId() const { return endCornerId_; }

private:
    std::string id_;
    std::string startCornerId_;
    std::string endCornerId_;
};

} // namespace Topology2D
