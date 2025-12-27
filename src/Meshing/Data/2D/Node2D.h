#pragma once

#include <string>

#include "Common/Types.h"

namespace Meshing
{

/**
 * @brief A mesh node in 2D parametric space
 */
class Node2D
{
public:
    explicit Node2D(const Point2D& coordinates);

    const Point2D& getCoordinates() const { return coordinates_; }
    void setCoordinates(const Point2D& coords) { coordinates_ = coords; }

    bool isBoundary() const { return isBoundary_; }
    void setBoundary(bool boundary) { isBoundary_ = boundary; }

    void setGeometryId(const std::string& id) { geometryId_ = id; }
    const std::string& getGeometryId() const { return geometryId_; }

private:
    Point2D coordinates_;
    bool isBoundary_ = false;
    std::string geometryId_;
};

} // namespace Meshing
