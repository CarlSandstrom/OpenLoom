#pragma once

#include "Common/Types.h"

#include <optional>

namespace Geometry3D
{
class ISurface3D;
} // namespace Geometry3D

namespace Meshing
{

class SurfaceProjector
{
public:
    SurfaceProjector() = default;

    double signedDistance(const Point3D& point, const Geometry3D::ISurface3D& surface) const;

    bool crossesSurface(const Point3D& c1,
                        const Point3D& c2,
                        const Geometry3D::ISurface3D& surface) const;

    std::optional<Point3D> projectToSurface(const Point3D& point,
                                            const Geometry3D::ISurface3D& surface) const;

private:

    static constexpr double NEAR_TANGENT_RELATIVE_TOLERANCE = 1e-10;

    double computeSurfaceDiameter(const Geometry3D::ISurface3D& surface) const;
};

} // namespace Meshing
