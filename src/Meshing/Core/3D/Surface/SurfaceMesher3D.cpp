#include "Meshing/Core/3D/Surface/SurfaceMesher3D.h"

#include "Meshing/Core/3D/RCDT/RCDTMesher.h"
#include "Topology/Topology3D.h"

namespace Meshing
{

namespace
{

bool hasPeriodicOrSeamSurfaces(const Topology3D::Topology3D& topology)
{
    return !topology.getSeamCollection().empty();
}

SurfaceMeshingStrategy resolveStrategy(SurfaceMeshingStrategy requested,
                                       const Topology3D::Topology3D& topology)
{
    if (requested != SurfaceMeshingStrategy::Auto)
        return requested;
    return hasPeriodicOrSeamSurfaces(topology) ? SurfaceMeshingStrategy::AmbientRCDT
                                               : SurfaceMeshingStrategy::PerFaceUV;
}

} // namespace

SurfaceMesher3D::SurfaceMesher3D(const Geometry3D::GeometryCollection3D& geometry,
                                 const Topology3D::Topology3D& topology,
                                 Geometry3D::DiscretizationSettings3D discretizationSettings,
                                 SurfaceMesh3DQualitySettings qualitySettings,
                                 SurfaceMeshingStrategy strategy) :
    strategy_(resolveStrategy(strategy, topology))
{
    if (strategy_ == SurfaceMeshingStrategy::AmbientRCDT)
    {
        rcdtMesher_ = std::make_unique<RCDTMesher>(
            geometry, topology, std::move(discretizationSettings), std::move(qualitySettings));
    }
    else
    {
        uvContext_.emplace(geometry, topology, std::move(discretizationSettings), std::move(qualitySettings));
    }
}

SurfaceMesher3D::~SurfaceMesher3D() = default;

SurfaceMesher3D::SurfaceMesher3D(SurfaceMesher3D&&) noexcept = default;
SurfaceMesher3D& SurfaceMesher3D::operator=(SurfaceMesher3D&&) noexcept = default;

SurfaceMesh3D SurfaceMesher3D::mesh()
{
    if (rcdtMesher_)
        return rcdtMesher_->mesh();

    uvContext_->refineSurfaces();
    return uvContext_->buildSurfaceMesh();
}

} // namespace Meshing
