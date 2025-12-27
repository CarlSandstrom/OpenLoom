#pragma once

#include <string>

#include "Common/Types.h"

namespace Geometry2D
{

/**
 * @brief Abstract interface for geometric vertices/corners
 */
class ICorner2D
{
public:
    virtual ~ICorner2D() = default;

    virtual Meshing::Point2D getPoint() const = 0;
    virtual std::string getId() const = 0;
};

} // namespace Geometry2D
