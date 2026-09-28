#pragma once

#include <string>

namespace Geometry3D
{

/**
 * @brief Abstract interface for geometric volumes (solid bodies / material
 * phases).
 *
 * The point classification it used to offer served only the retired centroid
 * phase test (OPE-186); what remains is the volume's identity in the
 * geometry collection.
 */
class IVolume3D
{
public:
    virtual ~IVolume3D() = default;

    virtual std::string getId() const = 0;
};

} // namespace Geometry3D
