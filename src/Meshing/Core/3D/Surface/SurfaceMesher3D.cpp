#include "Meshing/Core/3D/Surface/SurfaceMesher3D.h"

#include "Meshing/Core/3D/RCDT/RCDTMesher.h"

#include <utility>

namespace Meshing
{

SurfaceMesher3D::SurfaceMesher3D(const Geometry3D::GeometryCollection3D& geometry,
                                 const Topology3D::Topology3D& topology,
                                 Geometry3D::DiscretizationSettings3D discretizationSettings,
                                 SurfaceMesh3DQualitySettings qualitySettings,
                                 std::optional<SizingFieldSettings3D> sizingFieldSettings) :
    implementation_(std::make_unique<RCDTMesher>(geometry,
                                                 topology,
                                                 std::move(discretizationSettings),
                                                 std::move(qualitySettings),
                                                 std::move(sizingFieldSettings)))
{
}

SurfaceMesher3D::~SurfaceMesher3D() = default;

SurfaceMesher3D::SurfaceMesher3D(SurfaceMesher3D&&) noexcept = default;
SurfaceMesher3D& SurfaceMesher3D::operator=(SurfaceMesher3D&&) noexcept = default;

void SurfaceMesher3D::mesh()
{
    implementation_->meshSurface();
}

const SurfaceMesh3D* SurfaceMesher3D::getMesh() const
{
    return implementation_->getSurfaceMesh();
}

} // namespace Meshing
