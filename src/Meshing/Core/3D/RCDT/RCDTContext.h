#pragma once

#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/RCDT/RCDTQualitySettings.h"
#include "Meshing/Data/3D/SurfaceMesh3D.h"

#include <memory>

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

class MeshingContext3D;
class RestrictedTriangulation;

/**
 * @brief Orchestrates the three phases of ambient-space RCDT surface meshing.
 *
 * Analogous to SurfaceMeshingContext3D but operates in 3D ambient space rather
 * than per-face UV space.
 *
 * Usage:
 *   RCDTContext context(geometry, topology, discSettings, qualitySettings);
 *   context.buildInitial();
 *   context.refine();
 *   SurfaceMesh3D result = context.buildSurfaceMesh();
 */
class RCDTContext
{
public:
    RCDTContext(const Geometry3D::GeometryCollection3D& geometry,
                const Topology3D::Topology3D& topology,
                const Geometry3D::DiscretizationSettings3D& discretizationSettings = {},
                const RCDTQualitySettings& qualitySettings = {});

    ~RCDTContext();

    // Prevent copying
    RCDTContext(const RCDTContext&) = delete;
    RCDTContext& operator=(const RCDTContext&) = delete;

    // Allow moving
    RCDTContext(RCDTContext&&) noexcept;
    RCDTContext& operator=(RCDTContext&&) noexcept;

    /**
     * @brief Phase 1: discretize boundary, run Delaunay3D, build RestrictedTriangulation
     * and CurveSegmentManager.
     */
    void buildInitial();

    /**
     * @brief Phase 2: refine restricted triangulation to meet quality criteria.
     *
     * Must be called after buildInitial().
     */
    void refine();

    /**
     * @brief Phase 3: assemble and return the final SurfaceMesh3D.
     *
     * Must be called after refine().
     */
    SurfaceMesh3D buildSurfaceMesh() const;

private:
    const Geometry3D::GeometryCollection3D* geometry_;
    const Topology3D::Topology3D* topology_;
    Geometry3D::DiscretizationSettings3D discretizationSettings_;
    RCDTQualitySettings qualitySettings_;

    std::unique_ptr<MeshingContext3D> meshingContext_;
    std::unique_ptr<RestrictedTriangulation> restrictedTriangulation_;
};

} // namespace Meshing
