#pragma once

#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/General/SizingFieldBuilder3D.h"
#include "Meshing/Data/3D/SurfaceMesh3D.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"

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

class ISurfaceMesher3D;

/**
 * @brief Top-level surface mesher for 3D CAD geometry.
 *
 * Thin wrapper around an ISurfaceMesher3D implementation (currently always
 * RCDTMesher, the ambient RCDT pipeline). As with VolumeMesher3D, there is
 * no strategy enum: only one algorithm exists today. The ISurfaceMesher3D
 * interface is the extensibility point for future algorithms.
 *
 * Usage:
 * @code
 *   SurfaceMesher3D mesher(geometry, topology, discretizationSettings, qualitySettings);
 *   SurfaceMesh3D result = mesher.mesh();
 * @endcode
 */
class SurfaceMesher3D
{
public:
    /// sizingFieldSettings, when set, bounds boundary-discretization segment
    /// length by h(x) as well as by tangent angle (see
    /// BoundaryDiscretizer3D). Off by default.
    SurfaceMesher3D(const Geometry3D::GeometryCollection3D& geometry,
                    const Topology3D::Topology3D& topology,
                    Geometry3D::DiscretizationSettings3D discretizationSettings = {},
                    SurfaceMesh3DQualitySettings qualitySettings = {},
                    std::optional<SizingFieldSettings3D> sizingFieldSettings = std::nullopt);

    ~SurfaceMesher3D();

    // Prevent copying
    SurfaceMesher3D(const SurfaceMesher3D&) = delete;
    SurfaceMesher3D& operator=(const SurfaceMesher3D&) = delete;

    // Allow moving
    SurfaceMesher3D(SurfaceMesher3D&&) noexcept;
    SurfaceMesher3D& operator=(SurfaceMesher3D&&) noexcept;

    /// Runs the surface meshing pipeline. May only be called once per instance.
    SurfaceMesh3D mesh();

private:
    std::unique_ptr<ISurfaceMesher3D> implementation_;
};

} // namespace Meshing
