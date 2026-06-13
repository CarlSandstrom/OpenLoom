#pragma once

#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/RCDT/RCDTQualitySettings.h"

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

class RCDTContext
{
public:
    RCDTContext(const Geometry3D::GeometryCollection3D& geometry,
                const Topology3D::Topology3D& topology,
                const Geometry3D::DiscretizationSettings3D& discretizationSettings = {},
                const RCDTQualitySettings& qualitySettings = {});

    ~RCDTContext();

    RCDTContext(const RCDTContext&) = delete;
    RCDTContext& operator=(const RCDTContext&) = delete;

    RCDTContext(RCDTContext&&) noexcept;
    RCDTContext& operator=(RCDTContext&&) noexcept;

    const Geometry3D::GeometryCollection3D& getGeometry() const { return *geometry_; }
    const Topology3D::Topology3D& getTopology() const { return *topology_; }
    const Geometry3D::DiscretizationSettings3D& getDiscretizationSettings() const { return discretizationSettings_; }
    const RCDTQualitySettings& getQualitySettings() const { return qualitySettings_; }

    MeshingContext3D& getMeshingContext();
    const MeshingContext3D& getMeshingContext() const;

    RestrictedTriangulation& getRestrictedTriangulation();
    const RestrictedTriangulation& getRestrictedTriangulation() const;
    void setRestrictedTriangulation(std::unique_ptr<RestrictedTriangulation> restrictedTriangulation);

private:
    const Geometry3D::GeometryCollection3D* geometry_;
    const Topology3D::Topology3D* topology_;
    Geometry3D::DiscretizationSettings3D discretizationSettings_;
    RCDTQualitySettings qualitySettings_;

    std::unique_ptr<MeshingContext3D> meshingContext_;
    std::unique_ptr<RestrictedTriangulation> restrictedTriangulation_;
};

} // namespace Meshing
