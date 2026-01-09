#pragma once

#include <string>
#include <vector>

#include "Common/Types.h"

namespace Meshing
{

/**
 * @brief A mesh node in 2D parametric space
 *
 * Nodes are either interior nodes (no edge parameters) or boundary nodes
 * (with edge parameters and geometry IDs). Boundary nodes can be associated
 * with multiple edges (e.g., corners where edges meet). The isBoundary()
 * method returns true if the node has any edge parameters.
 */
class Node2D
{
public:
    /// Construct an interior node
    explicit Node2D(const Point2D& coordinates);

    /// Construct a boundary node with edge parameters and geometry IDs
    Node2D(const Point2D& coordinates, const std::vector<double>& edgeParameters, const std::vector<std::string>& geometryIds);

    const Point2D& getCoordinates() const { return coordinates_; }
    void setCoordinates(const Point2D& coords) { coordinates_ = coords; }

    bool isBoundary() const { return !edgeParameters_.empty(); }

    const std::vector<std::string>& getGeometryIds() const { return geometryIds_; }

    const std::vector<double>& getEdgeParameters() const { return edgeParameters_; }

private:
    Point2D coordinates_;
    std::vector<double> edgeParameters_;
    std::vector<std::string> geometryIds_;
};

} // namespace Meshing
