#pragma once

#include <string>
#include <vector>

#include "Common/Types.h"

namespace Meshing
{

class Node2D
{
public:
    /// Construct an interior node
    explicit Node2D(const Point2D& coordinates);

    /// Construct a boundary node with geometry IDs
    Node2D(const Point2D& coordinates, const std::vector<std::string>& geometryIds);

    const Point2D& getCoordinates() const { return coordinates_; }
    void setCoordinates(const Point2D& coords) { coordinates_ = coords; }

    bool isBoundary() const { return !geometryIds_.empty(); }

    const std::vector<std::string>& getGeometryIds() const { return geometryIds_; }

private:
    Point2D coordinates_;
    std::vector<std::string> geometryIds_;
};

} // namespace Meshing
