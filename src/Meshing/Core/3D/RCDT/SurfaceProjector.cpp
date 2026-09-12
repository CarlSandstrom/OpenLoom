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
// Provenance, as far as it can be established: the value arrived with this
// file's first commit (5ad42cc, OPE-128) carrying no derivation, as the
// near-tangent guard of the since-deleted crossesSurface(). Nothing in the
// history ties it to a measurement or to a property of any model, and it has
// never been changed. Note that OPE-150 (f6cb166) did NOT touch this one --
// the tolerance it removed was projectToSurface()'s separate 1e-3
// maximumProjectionGap_. Treat 1e-10 as unexplained, not as calibrated.
constexpr double NEAR_TANGENT_RELATIVE_TOLERANCE = 1e-10;

// Halvings findSurfaceCrossing() performs once it has a sign change to bracket.
//
// Provenance: arrived with findSurfaceCrossing() itself (7d15542) with no
// derivation recorded, and has never been changed. What the number buys is at
// least arithmetically fixed: 30 halvings narrow the bracket to 2^-30, about
// 1e-9, of the dual edge's length, and cost 30 CAD projections per insertion
// point -- this loop is why RestrictedTriangleRefiner computes the crossing
// for one bad triangle at a time rather than for all of them.
constexpr int BISECTION_ITERATIONS = 30;

/// The longest distance between any two corners of the surface's untrimmed
/// parameter rectangle — a cheap stand-in for the surface's size, used to turn
/// a relative tolerance into an absolute one. Falls back to 1.0 for a
/// degenerate surface whose four corners coincide.
double computeSurfaceDiameter(const Geometry3D::ISurface3D& surface)
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
    const auto uv = surface.projectPointToUnderlyingSurface(point);
    if (!uv)
        return 0.0;

    const Point3D surfacePoint = surface.getPoint(uv->x(), uv->y());
    const Vector3D normal = surface.getNormal(uv->x(), uv->y());

    return (point - surfacePoint).dot(normal);
}

std::optional<Point3D> projectToSurface(const Point3D& point,
                                        const Geometry3D::ISurface3D& surface)
{
    const auto uv = surface.projectPointToUnderlyingSurface(point);
    if (!uv)
        return std::nullopt;

    return surface.getPoint(uv->x(), uv->y());
}

std::optional<Point3D> findSurfaceCrossing(const Point3D& segmentStart,
                                           const Point3D& segmentEnd,
                                           const Geometry3D::ISurface3D& surface)
{
    const double distanceAtStart = signedDistance(segmentStart, surface);
    const double distanceAtEnd = signedDistance(segmentEnd, surface);

    const double tangentGuard = NEAR_TANGENT_RELATIVE_TOLERANCE * computeSurfaceDiameter(surface);

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
        const Point3D midpoint = 0.5 * (negativeEnd + positiveEnd);
        if (signedDistance(midpoint, surface) < 0.0)
            negativeEnd = midpoint;
        else
            positiveEnd = midpoint;
    }

    return 0.5 * (negativeEnd + positiveEnd);
}

} // namespace SurfaceProjector

} // namespace Meshing
