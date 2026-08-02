#pragma once

namespace Meshing
{
struct SurfaceMesh3D;

/**
 * @brief Strategy interface for algorithms that mesh a CAD solid's boundary surfaces.
 *
 * Implemented today by RCDTMesher; lets a top-level dispatcher (SurfaceMesher3D)
 * hold an implementation without depending on which algorithm it is.
 */
class ISurfaceMesher3D
{
public:
    virtual ~ISurfaceMesher3D() = default;

    virtual SurfaceMesh3D meshSurface() = 0;
};

} // namespace Meshing
