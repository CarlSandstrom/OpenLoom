#pragma once

#include <string>

#include "Common/Types.h"

namespace Geometry2D
{

/**
 * @brief A corner (vertex) in 2D parametric space
 *
 * Represents a point in the (u,v) parametric domain of a surface.
 */
class Corner2D
{
public:
    Corner2D(const std::string& id, const Meshing::Point2D& point);

    const std::string& getId() const { return id_; }
    const Meshing::Point2D& getPoint() const { return point_; }

private:
    std::string id_;
    Meshing::Point2D point_;
};

} // namespace Geometry2D
