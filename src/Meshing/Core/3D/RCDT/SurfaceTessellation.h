#pragma once

#include "Common/Types.h"

#include <array>
#include <cstddef>
#include <vector>

namespace Geometry3D
{
class ISurface3D;
} // namespace Geometry3D

namespace Meshing
{

/**
 * @brief A discrete triangulated approximation of one surface's trimmed
 * patch, used as an exact classification oracle.
 *
 * RestrictedTriangulation needs to know, robustly, whether a given segment
 * crosses a surface. Testing that directly against the surface's own
 * continuous (possibly curved) geometry requires a floating-point
 * near-tangent tolerance, which can misclassify a segment whose endpoints
 * are only fractions of a unit from the surface (see OPE-169). Reducing the
 * question to "does this segment cross any triangle in a fixed tessellation
 * of the surface" turns it into a purely point-based predicate
 * (RobustPredicates3D::segmentCrossesTriangle), evaluated exactly regardless
 * of how close to degenerate the input is.
 *
 * Built entirely from the ISurface3D interface every backend already
 * implements (getPoint(), getParameterBounds(), isPointWithinTrimmedBoundary())
 * — no CAD-kernel-specific tessellator required, so this works identically
 * for a flat plane or a NURBS patch, and doesn't tie RCDT to OCC.
 */
class SurfaceTessellation
{
public:
    /// Builds a UV-grid tessellation of surface's trimmed patch: samples a
    /// (samplesPerDirection + 1) x (samplesPerDirection + 1) grid of points
    /// across the surface's parameter bounds, and adds the two triangles of
    /// any grid cell with at least one corner within the trimmed boundary. A
    /// cell isn't clipped to the exact trim curve, so the tessellation can
    /// extend up to one grid cell past the true trimmed patch near its edge
    /// — harmless here, since callers (RestrictedTriangulation) separately
    /// check that the points they care about are within the true trim
    /// boundary; this tessellation only needs to not have gaps.
    void build(const Geometry3D::ISurface3D& surface, size_t samplesPerDirection);

    /// Whether segment (a, b) crosses this tessellation — exact, checking
    /// RobustPredicates3D::segmentCrossesTriangle against every triangle.
    bool crossesSurface(const Point3D& a, const Point3D& b) const;

private:
    std::vector<std::array<Point3D, 3>> triangles_;
};

} // namespace Meshing
