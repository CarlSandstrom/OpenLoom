#include "Meshing/Core/3D/RCDT/RCDTContext.h"

#include "Meshing/Core/3D/General/MeshingContext3D.h"
#include "Meshing/Core/3D/RCDT/RestrictedTriangulation.h"

namespace Meshing
{

RCDTContext::RCDTContext(const Geometry3D::GeometryCollection3D& geometry,
                         const Topology3D::Topology3D& topology,
                         const Geometry3D::DiscretizationSettings3D& discretizationSettings,
                         const RCDTQualitySettings& qualitySettings) :
    geometry_(&geometry),
    topology_(&topology),
    discretizationSettings_(discretizationSettings),
    qualitySettings_(qualitySettings),
    meshingContext_(std::make_unique<MeshingContext3D>(geometry, topology))
{
}

RCDTContext::~RCDTContext() = default;
RCDTContext::RCDTContext(RCDTContext&&) noexcept = default;
RCDTContext& RCDTContext::operator=(RCDTContext&&) noexcept = default;

MeshingContext3D& RCDTContext::getMeshingContext()
{
    return *meshingContext_;
}

const MeshingContext3D& RCDTContext::getMeshingContext() const
{
    return *meshingContext_;
}

RestrictedTriangulation& RCDTContext::getRestrictedTriangulation()
{
    return *restrictedTriangulation_;
}

const RestrictedTriangulation& RCDTContext::getRestrictedTriangulation() const
{
    return *restrictedTriangulation_;
}

void RCDTContext::setRestrictedTriangulation(std::unique_ptr<RestrictedTriangulation> restrictedTriangulation)
{
    restrictedTriangulation_ = std::move(restrictedTriangulation);
}

} // namespace Meshing
