#pragma once

#include <optional>
#include <string>

#include "Common/Types.h"

namespace Meshing
{

/**
 * @brief A mesh node in 2D parametric space
 *
 * Nodes are either interior nodes (no edge parameter) or boundary nodes
 * (with edge parameter and geometry ID). The isBoundary() method returns
 * true if the node has an edge parameter.
 */
class Node2D
{
public:
    /// Construct an interior node
    explicit Node2D(const Point2D& coordinates);

    /// Construct a boundary node with edge parameter and geometry ID
    Node2D(const Point2D& coordinates, double edgeParameter, const std::string& geometryId);

    const Point2D& getCoordinates() const { return coordinates_; }
    void setCoordinates(const Point2D& coords) { coordinates_ = coords; }

    bool isBoundary() const { return edgeParameter_.has_value(); }

    const std::string& getGeometryId() const { return geometryId_; }

    std::optional<double> getEdgeParameter() const { return edgeParameter_; }

private:
    Point2D coordinates_;
    std::optional<double> edgeParameter_;
    std::string geometryId_;
};

} // namespace Meshing
