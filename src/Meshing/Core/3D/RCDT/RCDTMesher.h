#pragma once

#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/RCDT/RCDTQualitySettings.h"
#include "Meshing/Data/3D/SurfaceMesh3D.h"

#include <array>
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

class RCDTMesher
{
public:
    RCDTMesher(const Geometry3D::GeometryCollection3D& geometry,
               const Topology3D::Topology3D& topology,
               Geometry3D::DiscretizationSettings3D discretizationSettings = {},
               RCDTQualitySettings qualitySettings = {});

    ~RCDTMesher();

    RCDTMesher(const RCDTMesher&) = delete;
    RCDTMesher& operator=(const RCDTMesher&) = delete;

    RCDTMesher(RCDTMesher&&) noexcept;
    RCDTMesher& operator=(RCDTMesher&&) noexcept;

    SurfaceMesh3D mesh();

    const MeshingContext3D& getMeshingContext() const;

private:
    const Geometry3D::GeometryCollection3D* geometry_;
    const Topology3D::Topology3D* topology_;
    Geometry3D::DiscretizationSettings3D discretizationSettings_;
    RCDTQualitySettings qualitySettings_;

    std::unique_ptr<MeshingContext3D> meshingContext_;
    std::unique_ptr<RestrictedTriangulation> restrictedTriangulation_;

    // Node IDs of the super-tetrahedron used to seed the initial Delaunay
    // triangulation. Kept alive through refine() -- see Delaunay3D's class
    // documentation for why -- and removed once refinement completes.
    std::array<size_t, 4> boundingNodeIds_{};

    void buildInitial();
    void refine();
    void removeBoundingTetrahedron();
    SurfaceMesh3D buildSurfaceMesh() const;
};

} // namespace Meshing
