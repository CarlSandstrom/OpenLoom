#pragma once

#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Data/3D/SurfaceMesh3D.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"
#include "Meshing/Data/3D/VolumeMesh3D.h"
#include "Meshing/Interfaces/ISurfaceMesher3D.h"
#include "Meshing/Interfaces/IVolumeMesher3D.h"

#include "Meshing/Core/3D/General/SizingField3D.h"
#include "Meshing/Core/3D/General/SizingFieldBuilder3D.h"

#include <array>
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

class MeshingContext3D;
class RestrictedTriangulation;

/**
 * @brief Ambient-space RCDT mesher: implements both ISurfaceMesher3D and
 * IVolumeMesher3D, since both share the same underlying ambient
 * tetrahedralization + restriction + refinement pipeline — meshSurface()
 * and meshVolume() differ only in their final extraction step (and, once
 * tet-quality refinement lands, whether that third refinement priority runs).
 */
class RCDTMesher : public ISurfaceMesher3D, public IVolumeMesher3D
{
public:
    /// sizingFieldSettings, when set, bounds boundary-discretization segment
    /// length by h(x) in addition to the tangent-angle criterion -- see
    /// BoundaryDiscretizer3D's class comment for why the angle criterion
    /// alone cannot bound length. Off by default.
    RCDTMesher(const Geometry3D::GeometryCollection3D& geometry,
               const Topology3D::Topology3D& topology,
               Geometry3D::DiscretizationSettings3D discretizationSettings = {},
               SurfaceMesh3DQualitySettings qualitySettings = {},
               std::optional<SizingFieldSettings3D> sizingFieldSettings = std::nullopt);

    ~RCDTMesher();

    RCDTMesher(const RCDTMesher&) = delete;
    RCDTMesher& operator=(const RCDTMesher&) = delete;

    RCDTMesher(RCDTMesher&&) noexcept;
    RCDTMesher& operator=(RCDTMesher&&) noexcept;

    SurfaceMesh3D meshSurface() override;
    VolumeMesh3D meshVolume() override;

    const MeshingContext3D& getMeshingContext() const;

private:
    const Geometry3D::GeometryCollection3D* geometry_;
    const Topology3D::Topology3D* topology_;
    Geometry3D::DiscretizationSettings3D discretizationSettings_;
    SurfaceMesh3DQualitySettings qualitySettings_;
    std::optional<SizingFieldSettings3D> sizingFieldSettings_;

    /// h(x), built once in buildInitial() when sizingFieldSettings_ is set.
    /// Shared by boundary discretization and the minimumEdgeLength floor --
    /// the point of OPE-181 is that those read the same field.
    std::optional<SizingField3D> sizingField_;

    std::unique_ptr<MeshingContext3D> meshingContext_;
    std::unique_ptr<RestrictedTriangulation> restrictedTriangulation_;

    // Node IDs of the super-tetrahedron used to seed the initial Delaunay
    // triangulation. Kept alive through refine() -- see Delaunay3D's class
    // documentation for why -- and removed once refinement completes.
    std::array<size_t, 4> boundingNodeIds_{};

    void buildInitial();

    /// includeTetQualityRefinement enables RCDTRefiner's priority-3 (bad
    /// tetrahedra) refinement pass -- meshVolume() passes true, meshSurface()
    /// passes false so it never pays for volume-quality refinement it has no
    /// use for.
    void refine(bool includeTetQualityRefinement);
    void removeBoundingTetrahedron();

    /// Shared build -> refine -> remove-supertet -> smooth pipeline, common to
    /// both meshSurface() and meshVolume(). Returns the (possibly smoothed)
    /// surface mesh; meshVolume() only needs it for the smoother's triangle
    /// adjacency and discards it once smoothing has synced back to the live
    /// mesh (see buildVolumeMesh()).
    SurfaceMesh3D runPipeline(bool includeTetQualityRefinement);

    SurfaceMesh3D buildSurfaceMesh() const;
    VolumeMesh3D buildVolumeMesh() const;
};

} // namespace Meshing
