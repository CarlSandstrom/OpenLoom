#pragma once

#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/General/SizingFieldBuilder3D.h"
#include "Meshing/Core/3D/Surface/SurfaceMeshingContext3D.h"
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

class RCDTMesher;

/// Which surface meshing pipeline SurfaceMesher3D runs.
enum class SurfaceMeshingStrategy
{
    /// Per-face UV-space triangulation (SurfaceMeshingContext3D). Legacy
    /// pipeline; cannot represent periodic/seam surfaces correctly.
    PerFaceUV,

    /// Ambient-space Restricted Constrained Delaunay Triangulation (RCDTMesher).
    AmbientRCDT,

    /// PerFaceUV unless the topology has periodic/seam surfaces, in which
    /// case AmbientRCDT.
    Auto
};

/**
 * @brief Top-level surface mesher for 3D CAD geometry.
 *
 * Dispatches to one of two pipelines (see SurfaceMeshingStrategy) and
 * returns a conforming SurfaceMesh3D. Under PerFaceUV:
 *   S1 — Boundary discretization and per-face UV triangulation (run at construction)
 *   S2 — Shewchuk angle-quality refinement in UV space with twin edge synchronisation
 *   S3 — Optional chord-deviation pass for geometric fidelity
 * Under AmbientRCDT, RCDTMesher runs its own build/refine/smooth pipeline
 * entirely within mesh().
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
    /// sizingFieldSettings, when set, bounds boundary-discretization segment
    /// length by h(x) as well as by tangent angle (see
    /// BoundaryDiscretizer3D). Off by default, and honoured only under
    /// AmbientRCDT -- the legacy PerFaceUV pipeline ignores it.
    SurfaceMesher3D(const Geometry3D::GeometryCollection3D& geometry,
                    const Topology3D::Topology3D& topology,
                    Geometry3D::DiscretizationSettings3D discretizationSettings = {},
                    SurfaceMesh3DQualitySettings qualitySettings = {},
                    SurfaceMeshingStrategy strategy = SurfaceMeshingStrategy::Auto,
                    std::optional<SizingFieldSettings3D> sizingFieldSettings = std::nullopt);

    ~SurfaceMesher3D();

    // Prevent copying
    SurfaceMesher3D(const SurfaceMesher3D&) = delete;
    SurfaceMesher3D& operator=(const SurfaceMesher3D&) = delete;

    // Allow moving
    SurfaceMesher3D(SurfaceMesher3D&&) noexcept;
    SurfaceMesher3D& operator=(SurfaceMesher3D&&) noexcept;

    /// The strategy actually resolved to (Auto is resolved at construction).
    SurfaceMeshingStrategy getStrategy() const { return strategy_; }

    /**
     * @brief Run refinement and assemble the surface mesh.
     *
     * Under PerFaceUV, executes the S2–S3 refinement passes on all faces.
     * Under AmbientRCDT, runs RCDTMesher's full build/refine/smooth pipeline.
     * Either way, may only be called once per instance.
     */
    SurfaceMesh3D mesh();

private:
    SurfaceMeshingStrategy strategy_;
    std::optional<SurfaceMeshingContext3D> uvContext_;
    std::unique_ptr<RCDTMesher> rcdtMesher_;
};

} // namespace Meshing
