#pragma once

#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/General/SizingFieldBuilder3D.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"
#include "Meshing/Data/3D/VolumeMesh3D.h"

#include <memory>
#include <optional>

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

class IVolumeMesher3D;

/**
 * @brief Top-level volume mesher for 3D CAD geometry.
 *
 * Thin wrapper around an IVolumeMesher3D implementation (currently always
 * RCDTMesher, the ambient RCDT pipeline). Unlike SurfaceMesher3D, this has
 * no strategy enum: only one algorithm exists today, so a dispatch enum
 * would just be padding. The IVolumeMesher3D interface is the extensibility
 * point for future algorithms.
 *
 * Usage:
 * @code
 *   VolumeMesher3D mesher(geometry, topology, discSettings, qualitySettings);
 *   VolumeMesh3D result = mesher.mesh();
 * @endcode
 */
class VolumeMesher3D
{
public:
    /// sizingFieldSettings, when set, bounds boundary-discretization segment
    /// length by h(x) as well as by tangent angle (see
    /// BoundaryDiscretizer3D). Off by default.
    VolumeMesher3D(const Geometry3D::GeometryCollection3D& geometry,
                   const Topology3D::Topology3D& topology,
                   Geometry3D::DiscretizationSettings3D discretizationSettings = {},
                   SurfaceMesh3DQualitySettings qualitySettings = {},
                   std::optional<SizingFieldSettings3D> sizingFieldSettings = std::nullopt);

    ~VolumeMesher3D();

    // Prevent copying
    VolumeMesher3D(const VolumeMesher3D&) = delete;
    VolumeMesher3D& operator=(const VolumeMesher3D&) = delete;

    // Allow moving
    VolumeMesher3D(VolumeMesher3D&&) noexcept;
    VolumeMesher3D& operator=(VolumeMesher3D&&) noexcept;

    /// Runs the volume meshing pipeline. May only be called once per instance.
    VolumeMesh3D mesh();

private:
    std::unique_ptr<IVolumeMesher3D> impl_;
};

} // namespace Meshing
