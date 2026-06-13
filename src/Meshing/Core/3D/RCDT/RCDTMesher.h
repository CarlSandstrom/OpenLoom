#pragma once

#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/RCDT/RCDTContext.h"
#include "Meshing/Core/3D/RCDT/RCDTQualitySettings.h"
#include "Meshing/Data/3D/SurfaceMesh3D.h"

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
    RCDTContext context_;

    void buildInitial();
    void refine();
    SurfaceMesh3D buildSurfaceMesh() const;
};

} // namespace Meshing
