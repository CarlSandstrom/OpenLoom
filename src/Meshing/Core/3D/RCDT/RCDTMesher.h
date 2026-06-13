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

private:
    const Geometry3D::GeometryCollection3D* geometry_;
    const Topology3D::Topology3D* topology_;
    Geometry3D::DiscretizationSettings3D discretizationSettings_;
    RCDTQualitySettings qualitySettings_;

    std::unique_ptr<MeshingContext3D> meshingContext_;
    std::unique_ptr<RestrictedTriangulation> restrictedTriangulation_;

    void buildInitial();
    void refine();
    SurfaceMesh3D buildSurfaceMesh() const;
};

} // namespace Meshing
