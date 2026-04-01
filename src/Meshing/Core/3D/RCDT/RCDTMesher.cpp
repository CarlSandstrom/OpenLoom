#include "Meshing/Core/3D/RCDT/RCDTMesher.h"

namespace Meshing
{

RCDTMesher::RCDTMesher(const Geometry3D::GeometryCollection3D& geometry,
                       const Topology3D::Topology3D& topology,
                       Geometry3D::DiscretizationSettings3D discretizationSettings,
                       RCDTQualitySettings qualitySettings) :
    context_(geometry, topology, discretizationSettings, qualitySettings)
{
}

RCDTMesher::~RCDTMesher() = default;
RCDTMesher::RCDTMesher(RCDTMesher&&) noexcept = default;
RCDTMesher& RCDTMesher::operator=(RCDTMesher&&) noexcept = default;

SurfaceMesh3D RCDTMesher::mesh()
{
    context_.buildInitial();
    context_.refine();
    return context_.buildSurfaceMesh();
}

} // namespace Meshing
