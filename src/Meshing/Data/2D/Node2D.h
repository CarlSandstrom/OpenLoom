#pragma once

#include "Common/Types.h"

namespace Meshing
{

class Node2D
{
public:
    explicit Node2D(const Point2D& coordinates);

    const Point2D& getCoordinates() const { return coordinates_; }
    void setCoordinates(const Point2D& coords) { coordinates_ = coords; }

private:
    Point2D coordinates_;
};

} // namespace Meshing
