#include "Meshing/Core/3D/RCDT/SurfaceProjector.h"

#include "Common/BoundingBox2D.h"
#include "Geometry/3D/Base/ISurface3D.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace Meshing
{

double SurfaceProjector::signedDistance(const Point3D& point,
                                        const Geometry3D::ISurface3D& surface) const
{
    const auto uv = surface.projectPointToUnderlyingSurface(point);
    if (!uv)
        return 0.0;

    const Point3D surfacePoint = surface.getPoint(uv->x(), uv->y());
    const auto normalArray = surface.getNormal(uv->x(), uv->y());
    const Point3D normal(normalArray[0], normalArray[1], normalArray[2]);

    return (point - surfacePoint).dot(normal);
}

std::optional<Point3D> SurfaceProjector::projectToSurface(
    const Point3D& point,
    const Geometry3D::ISurface3D& surface) const
{
    const auto uv = surface.projectPointToUnderlyingSurface(point);
    if (!uv)
        return std::nullopt;

    return surface.getPoint(uv->x(), uv->y());
}

std::optional<Point3D> SurfaceProjector::findSurfaceCrossing(
    const Point3D& c1,
    const Point3D& c2,
    const Geometry3D::ISurface3D& surface) const
{
    const double d1 = signedDistance(c1, surface);
    const double d2 = signedDistance(c2, surface);

    const double diameter = computeSurfaceDiameter(surface);
    const double tangentGuard = NEAR_TANGENT_RELATIVE_TOLERANCE * diameter;

    // Already within tangent tolerance of the surface across the whole
    // segment: the midpoint is as good an answer as bisecting further would
    // give, and there may be no clean sign change to bisect on.
    if (std::abs(d1) < tangentGuard && std::abs(d2) < tangentGuard)
        return 0.5 * (c1 + c2);

    if (d1 * d2 >= 0.0)
        return std::nullopt;

    Point3D negativeEnd = d1 < 0.0 ? c1 : c2;
    Point3D positiveEnd = d1 < 0.0 ? c2 : c1;

    for (int i = 0; i < BISECTION_ITERATIONS; ++i)
    {
        const Point3D midpoint = 0.5 * (negativeEnd + positiveEnd);
        const double midpointDistance = signedDistance(midpoint, surface);
        if (midpointDistance < 0.0)
            negativeEnd = midpoint;
        else
            positiveEnd = midpoint;
    }

    return 0.5 * (negativeEnd + positiveEnd);
}

double SurfaceProjector::computeSurfaceDiameter(const Geometry3D::ISurface3D& surface) const
{
    const auto bounds = surface.getParameterBounds();
    const double uMin = bounds.getUMin();
    const double uMax = bounds.getUMax();
    const double vMin = bounds.getVMin();
    const double vMax = bounds.getVMax();

    const std::array<Point3D, 4> corners = {
        surface.getPoint(uMin, vMin),
        surface.getPoint(uMax, vMin),
        surface.getPoint(uMin, vMax),
        surface.getPoint(uMax, vMax)
    };

    double maximumDistance = 0.0;
    for (int i = 0; i < 4; ++i)
        for (int j = i + 1; j < 4; ++j)
            maximumDistance = std::max(maximumDistance, (corners[i] - corners[j]).norm());

    return maximumDistance > 0.0 ? maximumDistance : 1.0;
}

} // namespace Meshing
