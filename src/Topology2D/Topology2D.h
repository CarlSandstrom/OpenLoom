#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "Corner2D.h"
#include "Edge2D.h"

namespace Topology2D
{

/**
 * @brief Topological structure for a 2D domain
 *
 * Holds the topological entities (corners and edges) that define the
 * boundaries of a 2D domain. Typically extracted from a 3D surface.
 */
class Topology2D
{
public:
    Topology2D(const std::unordered_map<std::string, Corner2D>& corners,
               const std::unordered_map<std::string, Edge2D>& edges,
               const std::vector<std::string>& boundaryEdgeLoop);

    // Entity access
    const Corner2D& getCorner(const std::string& id) const;
    const Edge2D& getEdge(const std::string& id) const;

    // Global queries
    std::vector<std::string> getAllCornerIds() const;
    std::vector<std::string> getAllEdgeIds() const;

    // Boundary loop (ordered list of edge IDs forming the closed boundary)
    const std::vector<std::string>& getBoundaryEdgeLoop() const { return boundaryEdgeLoop_; }

    size_t getCornerCount() const { return corners_.size(); }
    size_t getEdgeCount() const { return edges_.size(); }

private:
    std::unordered_map<std::string, Corner2D> corners_;
    std::unordered_map<std::string, Edge2D> edges_;
    std::vector<std::string> boundaryEdgeLoop_;
};

} // namespace Topology2D
