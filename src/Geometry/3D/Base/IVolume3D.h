#pragma once

#include <string>

namespace Geometry3D
{

/**
 * @brief Abstract interface for geometric volumes (solid bodies / material
 * phases), identified by its id in the geometry collection.
 */
class IVolume3D
{
public:
    virtual ~IVolume3D() = default;

    virtual std::string getId() const = 0;
};

} // namespace Geometry3D
