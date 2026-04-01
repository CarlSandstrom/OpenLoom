#pragma once

#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/RCDT/RCDTContext.h"
#include "Meshing/Core/3D/RCDT/RCDTQualitySettings.h"
#include "Meshing/Data/3D/SurfaceMesh3D.h"

namespace Geometry3D
{
class GeometryCollection3D;
} // namespace Geometry3D

namespace Topology3D
{
class Topology3D;
} // namespace Topology3D

namespace Meshing
{

/**
 * @brief Top-level ambient-space RCDT surface mesher.
 *
 * Runs the three RCDT phases and returns a conforming SurfaceMesh3D.
 * Constructor signature matches SurfaceMesher3D for straightforward substitution.
 *
 * Usage:
 * @code
 *   RCDTMesher mesher(geometry, topology, discSettings, qualitySettings);
 *   SurfaceMesh3D result = mesher.mesh();
 * @endcode
 */
class RCDTMesher
{
public:
    RCDTMesher(const Geometry3D::GeometryCollection3D& geometry,
               const Topology3D::Topology3D& topology,
               Geometry3D::DiscretizationSettings3D discretizationSettings = {},
               RCDTQualitySettings qualitySettings = {});

    ~RCDTMesher();

    // Prevent copying
    RCDTMesher(const RCDTMesher&) = delete;
    RCDTMesher& operator=(const RCDTMesher&) = delete;

    // Allow moving
    RCDTMesher(RCDTMesher&&) noexcept;
    RCDTMesher& operator=(RCDTMesher&&) noexcept;

    /**
     * @brief Run all three RCDT phases and return the surface mesh.
     *
     * May only be called once per instance.
     */
    SurfaceMesh3D mesh();

private:
    RCDTContext context_;
};

} // namespace Meshing
