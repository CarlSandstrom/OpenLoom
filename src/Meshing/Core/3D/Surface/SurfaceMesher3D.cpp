#include "Meshing/Core/3D/Surface/SurfaceMesher3D.h"

namespace Meshing
{

SurfaceMesher3D::SurfaceMesher3D(const Geometry3D::GeometryCollection3D& geometry,
                                 const Topology3D::Topology3D& topology,
                                 Geometry3D::DiscretizationSettings3D discretizationSettings,
                                 SurfaceMesh3DQualitySettings qualitySettings) :
    context_(geometry, topology, std::move(discretizationSettings), std::move(qualitySettings))
{
}

SurfaceMesher3D::~SurfaceMesher3D() = default;

SurfaceMesher3D::SurfaceMesher3D(SurfaceMesher3D&&) noexcept = default;
SurfaceMesher3D& SurfaceMesher3D::operator=(SurfaceMesher3D&&) noexcept = default;

SurfaceMesh3D SurfaceMesher3D::mesh()
{
    context_.refineSurfaces();
    return context_.buildSurfaceMesh();
}

} // namespace Meshing
