#pragma once

#include <string>

#include "Common/Types.h"

namespace Geometry3D
{

/**
 * @brief Abstract interface for geometric vertices/corners
 */
class ICorner3D
{
public:
    virtual ~ICorner3D() = default;

    virtual Meshing::Point3D getPoint() const = 0;
    virtual std::string getId() const = 0;
};

} // namespace Geometry3D