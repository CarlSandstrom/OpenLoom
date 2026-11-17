#include "Meshing/Core/3D/General/SurfaceProjector.h"

#include "Geometry/3D/Base/ISurface3D.h"

#include <Eigen/LU>

#include <cmath>

namespace Meshing
{

namespace SurfaceProjector
{

namespace
{

// findSurfaceCrossing() has converged once the point on the segment and the
// point on the surface are this fraction of the segment's length apart.
constexpr double NEWTON_RELATIVE_TOLERANCE = 1e-12;

// Newton converges quadratically from a seed within a tessellation cell of the
// crossing, so this many iterations without converging means it never will.
constexpr int NEWTON_MAXIMUM_ITERATIONS = 20;

} // namespace

double signedDistance(const Point3D& point, const Geometry3D::ISurface3D& surface)
{
    auto uv = surface.projectPointToUnderlyingSurface(point);
    if (!uv)
        return 0.0;

    Point3D surfacePoint = surface.getPoint(uv->x(), uv->y());
    Vector3D normal = surface.getNormal(uv->x(), uv->y());

    return (point - surfacePoint).dot(normal);
}

std::optional<Point3D> projectToSurface(const Point3D& point,
                                        const Geometry3D::ISurface3D& surface)
{
    auto uv = surface.projectPointToUnderlyingSurface(point);
    if (!uv)
        return std::nullopt;

    return surface.getPoint(uv->x(), uv->y());
}

std::optional<Point3D> findSurfaceCrossing(const Point3D& segmentStart,
                                           const Point3D& segmentEnd,
                                           const Geometry3D::ISurface3D& surface,
                                           const Point2D& seedUV)
{
    Vector3D direction = segmentEnd - segmentStart;
    double squaredLength = direction.squaredNorm();
    if (squaredLength <= 0.0)
        return std::nullopt;

    // Solve segmentStart + t * direction = S(u, v) for (t, u, v).
    double tolerance = NEWTON_RELATIVE_TOLERANCE * std::sqrt(squaredLength);
    Point2D uv = seedUV;
    double parameter = (surface.getPoint(uv.x(), uv.y()) - segmentStart).dot(direction) / squaredLength;
    for (int iteration = 0; iteration < NEWTON_MAXIMUM_ITERATIONS; ++iteration)
    {
        auto derivatives = surface.getFirstDerivatives(uv.x(), uv.y());
        Vector3D residual = segmentStart + parameter * direction - derivatives.point;
        if (residual.norm() <= tolerance)
        {
            if (parameter < 0.0 || parameter > 1.0)
                return std::nullopt;
            return segmentStart + parameter * direction;
        }

        Eigen::Matrix3d jacobian;
        jacobian << direction, -derivatives.derivativeU, -derivatives.derivativeV;
        auto decomposition = jacobian.fullPivLu();
        if (!decomposition.isInvertible())
            return std::nullopt;

        Eigen::Vector3d step = decomposition.solve(-residual);
        parameter += step[0];
        uv += step.tail<2>();
    }
    return std::nullopt;
}

} // namespace SurfaceProjector

} // namespace Meshing
