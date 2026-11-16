#pragma once

#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/General/SizingFieldBuilder3D.h"
#include "Meshing/Core/3D/RCDT/RestrictedFaceTypes.h"
#include "Meshing/Data/3D/SurfaceMesh3D.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"
#include "Meshing/Data/3D/VolumeMesh3D.h"
#include "Meshing/Interfaces/3D/ISurfaceMesher3D.h"
#include "Meshing/Interfaces/3D/IVolumeMesher3D.h"

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

/**
 * @brief Ambient-space RCDT mesher: implements both ISurfaceMesher3D and
 * IVolumeMesher3D on one pipeline, CGAL Mesh_3's design (OPE-186): a weighted
 * ambient tetrahedralization seeded with protecting balls
 * (ProtectingBallPlacer), facet refinement (SurfaceDelaunayRefiner) and, for a
 * volume, tetrahedron refinement below it (TetrahedronDelaunayRefiner).
 * meshSurface() and meshVolume() differ only in that level, in the
 * closed-boundary check a volume needs, and in their final extraction step.
 *
 * Each call builds its mesh from scratch and keeps the result, readable
 * through getSurfaceMesh() / getVolumeMesh() until the next call.
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

    RCDTMesher(const RCDTMesher&) = delete;
    RCDTMesher& operator=(const RCDTMesher&) = delete;

    void meshSurface() override;
    void meshVolume() override;

    const SurfaceMesh3D* getSurfaceMesh() const override;
    const VolumeMesh3D* getVolumeMesh() const override;

private:
    const Geometry3D::GeometryCollection3D* geometry_;
    const Topology3D::Topology3D* topology_;
    Geometry3D::DiscretizationSettings3D discretizationSettings_;
    SurfaceMesh3DQualitySettings qualitySettings_;
    std::optional<SizingFieldSettings3D> sizingFieldSettings_;
    std::unique_ptr<SurfaceMesh3D> surfaceMesh_;
    std::unique_ptr<VolumeMesh3D> volumeMesh_;

    /// Discretizes the boundary, resolves the size floor (qualitySettings_'s
    /// if set, otherwise MinimumEdgeLengthEstimator's), places the protecting
    /// balls and seeds the weighted ambient tetrahedralization with its curve
    /// segments. Returns the size floor.
    double buildInitialTriangulation(MeshingContext3D& context) const;

    /// Removes the ambient tetrahedra, then extracts and smooths the surface
    /// mesh into surfaceMesh_, writing a "smoothed" snapshot when smoothing ran.
    /// meshingVolume keeps smoothing from inverting tetrahedra.
    void buildSurfaceMesh(MeshingContext3D& context, const RestrictedFaceMap& restrictedFaces, bool meshingVolume);
};

} // namespace Meshing
