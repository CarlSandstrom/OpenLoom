#pragma once

#include <string>

#include "Common/Types.h"

namespace Geometry
{

/**
 * @brief Abstract interface for geometric vertices/corners
 */
class Corner
{
public:
    virtual ~Corner() = default;

    virtual Meshing::Point3D getPoint() const = 0;
    virtual std::string getId() const = 0;
};

} // namespace Geometry