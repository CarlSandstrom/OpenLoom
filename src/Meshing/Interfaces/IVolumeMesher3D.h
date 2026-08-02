#pragma once

namespace Meshing
{
struct VolumeMesh3D;

/**
 * @brief Strategy interface for algorithms that mesh a CAD solid's interior volume.
 *
 * Implemented today by RCDTMesher; lets a top-level dispatcher (VolumeMesher3D)
 * hold an implementation without depending on which algorithm it is.
 */
class IVolumeMesher3D
{
public:
    virtual ~IVolumeMesher3D() = default;

    virtual VolumeMesh3D meshVolume() = 0;
};

} // namespace Meshing
