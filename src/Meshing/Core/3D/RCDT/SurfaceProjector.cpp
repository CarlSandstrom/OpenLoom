#include "Meshing/Core/3D/RCDT/SurfaceProjector.h"

#include "Common/BoundingBox2D.h"
#include "Geometry/3D/Base/ISurface3D.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace Meshing
{

namespace SurfaceProjector
{

namespace
{

// Fraction of the surface's diameter within which findSurfaceCrossing() treats
// both segment endpoints as sitting ON the surface, and answers with the
// midpoint instead of bisecting.
//
// The value has no recorded derivation and has never been measured against a
// model; treat it as unexplained, not calibrated.
constexpr double NEAR_TANGENT_RELATIVE_TOLERANCE = 1e-10;

// Halvings findSurfaceCrossing() performs once it has a sign change to bracket.
//
// 30 halvings narrow the bracket to 2^-30, about 1e-9, of the dual edge's
// length, at 30 CAD projections per crossing, which is why
// WeightedDualRestriction bisects only within a few tessellation cells of the
// tessellation's crossing.
constexpr int BISECTION_ITERATIONS = 30;

/// The longest distance between any two corners of the surface's untrimmed
/// parameter rectangle — a cheap stand-in for the surface's size, used to turn
/// a relative tolerance into an absolute one. Falls back to 1.0 for a
/// degenerate surface whose four corners coincide.
double computeSurfaceDiameter(const Geometry3D::ISurface3D& surface)
{
    auto bounds = surface.getParameterBounds();
    double uMin = bounds.getUMin();
    double uMax = bounds.getUMax();
    double vMin = bounds.getVMin();
    double vMax = bounds.getVMax();

    std::array<Point3D, 4> corners = {
        surface.getPoint(uMin, vMin),
        surface.getPoint(uMax, vMin),
        surface.getPoint(uMin, vMax),
        surface.getPoint(uMax, vMax)};

    double maximumDistance = 0.0;
    for (int i = 0; i < 4; ++i)
        for (int j = i + 1; j < 4; ++j)
            maximumDistance = std::max(maximumDistance, (corners[i] - corners[j]).norm());

    return maximumDistance > 0.0 ? maximumDistance : 1.0;
}

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
                                           const Geometry3D::ISurface3D& surface)
{
    double distanceAtStart = signedDistance(segmentStart, surface);
    double distanceAtEnd = signedDistance(segmentEnd, surface);

    double tangentGuard = NEAR_TANGENT_RELATIVE_TOLERANCE * computeSurfaceDiameter(surface);

    // Already within tangent tolerance of the surface across the whole
    // segment: the midpoint is as good an answer as bisecting further would
    // give, and there may be no clean sign change to bisect on.
    if (std::abs(distanceAtStart) < tangentGuard && std::abs(distanceAtEnd) < tangentGuard)
        return 0.5 * (segmentStart + segmentEnd);

    if (distanceAtStart * distanceAtEnd >= 0.0)
        return std::nullopt;

    Point3D negativeEnd = distanceAtStart < 0.0 ? segmentStart : segmentEnd;
    Point3D positiveEnd = distanceAtStart < 0.0 ? segmentEnd : segmentStart;

    for (int iteration = 0; iteration < BISECTION_ITERATIONS; ++iteration)
    {
        Point3D midpoint = 0.5 * (negativeEnd + positiveEnd);
        if (signedDistance(midpoint, surface) < 0.0)
            negativeEnd = midpoint;
        else
            positiveEnd = midpoint;
    }

    return 0.5 * (negativeEnd + positiveEnd);
}

} // namespace SurfaceProjector

} // namespace Meshing
