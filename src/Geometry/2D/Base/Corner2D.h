#pragma once

#include <string>

#include "Common/Types.h"
#include "Geometry/2D/Base/ICorner2D.h"

namespace Geometry2D
{

/**
 * @brief A corner (vertex) in 2D parametric space
 *
 * Represents a point in the (u,v) parametric domain of a surface.
 */
class Corner2D : public ICorner2D
{
public:
    Corner2D(const std::string& id, const Meshing::Point2D& point);

    Meshing::Point2D getPoint() const override;
    std::string getId() const override;

private:
    std::string id_;
    Meshing::Point2D point_;
};

} // namespace Geometry2D
