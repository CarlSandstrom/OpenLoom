#pragma once

namespace Meshing
{
struct SurfaceMesh3D;

/**
 * @brief Strategy interface for algorithms that mesh a CAD solid's boundary surfaces.
 *
 * Implemented today by RCDTMesher; lets the top-level SurfaceMesher3D
 * hold an implementation without depending on which algorithm it is.
 */
class ISurfaceMesher3D
{
public:
    virtual ~ISurfaceMesher3D() = default;

    virtual void meshSurface() = 0;

    /// The mesh built by meshSurface(); nullptr until it has run.
    virtual const SurfaceMesh3D* getSurfaceMesh() const = 0;
};

} // namespace Meshing
