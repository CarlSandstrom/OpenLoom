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

    /// Finds where the surface crosses the segment [c1, c2] — the restricted
    /// Voronoi vertex for a face whose dual Voronoi edge runs between c1 and
    /// c2 (typically the circumcenters of the two tetrahedra sharing that
    /// face). Requires crossesSurface(c1, c2, surface) to hold; returns
    /// nullopt if it does not (no crossing to find). Uses bisection on
    /// signedDistance, so it works regardless of how curved the surface is
    /// between c1 and c2.
    std::optional<Point3D> findSurfaceCrossing(const Point3D& c1,
                                               const Point3D& c2,
                                               const Geometry3D::ISurface3D& surface) const;

private:

    static constexpr double NEAR_TANGENT_RELATIVE_TOLERANCE = 1e-10;
    static constexpr int BISECTION_ITERATIONS = 30;

    double computeSurfaceDiameter(const Geometry3D::ISurface3D& surface) const;
};

} // namespace Meshing
