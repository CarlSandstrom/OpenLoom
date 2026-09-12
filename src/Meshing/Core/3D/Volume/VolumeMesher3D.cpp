#include "Meshing/Core/3D/Volume/VolumeMesher3D.h"

#include "Meshing/Core/3D/RCDT/RCDTMesher.h"

namespace Meshing
{

VolumeMesher3D::VolumeMesher3D(const Geometry3D::GeometryCollection3D& geometry,
                               const Topology3D::Topology3D& topology,
                               Geometry3D::DiscretizationSettings3D discretizationSettings,
                               SurfaceMesh3DQualitySettings qualitySettings,
                               std::optional<SizingFieldSettings3D> sizingFieldSettings) :
    impl_(std::make_unique<RCDTMesher>(geometry,
                                       topology,
                                       std::move(discretizationSettings),
                                       std::move(qualitySettings),
                                       std::move(sizingFieldSettings)))
{
}

VolumeMesher3D::~VolumeMesher3D() = default;

VolumeMesher3D::VolumeMesher3D(VolumeMesher3D&&) noexcept = default;
VolumeMesher3D& VolumeMesher3D::operator=(VolumeMesher3D&&) noexcept = default;

VolumeMesh3D VolumeMesher3D::mesh()
{
    return impl_->meshVolume();
}

} // namespace Meshing
