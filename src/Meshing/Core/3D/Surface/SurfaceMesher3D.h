#pragma once

#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/Surface/SurfaceMeshingContext3D.h"
#include "Meshing/Data/3D/SurfaceMesh3D.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"

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
 * @brief Top-level surface mesher for 3D CAD geometry.
 *
 * Runs the full S1–S3 pipeline and returns a conforming SurfaceMesh3D:
 *   S1 — Boundary discretization and per-face UV triangulation (run at construction)
 *   S2 — Shewchuk angle-quality refinement in UV space with twin edge synchronisation
 *   S3 — Optional chord-deviation pass for geometric fidelity
 *
 * Usage:
 * @code
 *   SurfaceMesher3D mesher(geometry, topology, discSettings, qualitySettings);
 *   SurfaceMesh3D result = mesher.mesh();
 * @endcode
 */
class SurfaceMesher3D
{
public:
    SurfaceMesher3D(const Geometry3D::GeometryCollection3D& geometry,
                    const Topology3D::Topology3D& topology,
                    Geometry3D::DiscretizationSettings3D discretizationSettings = {},
                    SurfaceMesh3DQualitySettings qualitySettings = {});

    ~SurfaceMesher3D();

    // Prevent copying
    SurfaceMesher3D(const SurfaceMesher3D&) = delete;
    SurfaceMesher3D& operator=(const SurfaceMesher3D&) = delete;

    // Allow moving
    SurfaceMesher3D(SurfaceMesher3D&&) noexcept;
    SurfaceMesher3D& operator=(SurfaceMesher3D&&) noexcept;

    /**
     * @brief Run refinement and assemble the surface mesh.
     *
     * Executes the S2–S3 refinement passes on all faces, then assembles and
     * returns the full SurfaceMesh3D.  May only be called once per instance.
     */
    SurfaceMesh3D mesh();

private:
    SurfaceMeshingContext3D context_;
};

} // namespace Meshing
