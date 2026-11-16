#pragma once

#include "Topology2D/Corner2D.h"
#include "Topology2D/Edge2D.h"
#include <string>
#include <unordered_map>
#include <vector>

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
    // Constructor for a domain without holes
    Topology2D(const std::unordered_map<std::string, Corner2D>& corners,
               const std::unordered_map<std::string, Edge2D>& edges,
               const std::vector<std::string>& boundaryEdgeLoop);

    // Constructor with explicit hole loops
    Topology2D(const std::unordered_map<std::string, Corner2D>& corners,
               const std::unordered_map<std::string, Edge2D>& edges,
               const std::vector<std::string>& outerEdgeLoop,
               const std::vector<std::vector<std::string>>& holeEdgeLoops);

    const Edge2D& getEdge(const std::string& id) const;

    std::vector<std::string> getAllEdgeIds() const;

    const std::vector<std::string>& getOuterEdgeLoop() const;
    const std::vector<std::vector<std::string>>& getHoleEdgeLoops() const;

    size_t getCornerCount() const;
    size_t getEdgeCount() const;

private:
    std::unordered_map<std::string, Corner2D> corners_;
    std::unordered_map<std::string, Edge2D> edges_;
    std::vector<std::string> outerEdgeLoop_;
    std::vector<std::vector<std::string>> holeEdgeLoops_;
};

} // namespace Topology2D
