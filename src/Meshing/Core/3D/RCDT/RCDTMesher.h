#pragma once

#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/General/SizingFieldBuilder3D.h"
#include "Meshing/Core/3D/RCDT/RestrictedFaceTypes.h"
#include "Meshing/Data/3D/SurfaceMesh3D.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"
#include "Meshing/Data/3D/VolumeMesh3D.h"
#include "Meshing/Interfaces/ISurfaceMesher3D.h"
#include "Meshing/Interfaces/IVolumeMesher3D.h"

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
 * meshSurface() and meshVolume() differ only in that level and in their final
 * extraction step.
 *
 * Each call builds its own mesh from scratch and releases it on return;
 * nothing is kept between calls.
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

    SurfaceMesh3D meshSurface() override;
    VolumeMesh3D meshVolume() override;

private:
    const Geometry3D::GeometryCollection3D* geometry_;
    const Topology3D::Topology3D* topology_;
    Geometry3D::DiscretizationSettings3D discretizationSettings_;
    SurfaceMesh3DQualitySettings qualitySettings_;
    std::optional<SizingFieldSettings3D> sizingFieldSettings_;

    /// Discretizes the boundary, resolves the size floor (qualitySettings_'s
    /// if set, otherwise MinimumEdgeLengthEstimator's), places the protecting
    /// balls and seeds the weighted ambient tetrahedralization with its curve
    /// segments. Returns the size floor.
    double seedTriangulation(MeshingContext3D& context) const;

    /// Seed, then SurfaceDelaunayRefiner -- driven by TetrahedronDelaunayRefiner
    /// when meshingVolume -- then remove the ambient tetrahedra, extract and
    /// smooth the surface mesh. meshingVolume also keeps smoothing from
    /// inverting tetrahedra and makes a restricted boundary with holes an
    /// error. restrictedFaces receives the restricted facets for volume
    /// extraction; meshVolume() only needs the returned surface mesh for the
    /// smoother, which has synced its positions back into the live mesh.
    SurfaceMesh3D runPipeline(MeshingContext3D& context, RestrictedFaceMap& restrictedFaces, bool meshingVolume) const;
};

} // namespace Meshing
