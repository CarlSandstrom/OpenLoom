#pragma once

#include "Common/Types.h"
#include "Meshing/Core/3D/RCDT/TriangleSoupIndex.h"

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
 * implements (getPoint(), getParameterBounds(), isUVWithinTrimmedBoundary())
 * — no CAD-kernel-specific tessellator required, so this works identically
 * for a flat plane or a NURBS patch, and doesn't tie RCDT to OCC.
 *
 * This class is the UV sampling half of that job: where to place samples and
 * which cells to emit. The triangles it produces are held in a
 * TriangleSoupIndex, which knows nothing about surfaces and answers the
 * crossing query.
 */
class SurfaceTessellation
{
public:
    /// Builds a UV-grid tessellation of surface's trimmed patch at a
    /// resolution whose triangle edge lengths are at most targetCellSize —
    /// scaled the same way regardless of whether the surface is flat: a flat
    /// surface's triangles are exact at any resolution, but the jittered
    /// grid's own edge-coverage gap near the trim boundary is not, so
    /// resolution still needs to track targetCellSize there too (see the
    /// .cpp for the gap this fixed). The computed sample count is capped at
    /// MAXIMUM_SAMPLES_PER_DIRECTION to bound memory and cost.
    /// A cell isn't clipped to the exact trim curve, so the tessellation can
    /// extend up to one grid cell past the true trimmed patch near its edge
    /// — harmless here, since callers (RestrictedTriangulation) separately
    /// check that the points they care about are within the true trim
    /// boundary; this tessellation only needs to not have gaps.
    void build(const Geometry3D::ISurface3D& surface, double targetCellSize);

    /// Whether segment (a, b) crosses this tessellation — exact; see
    /// TriangleSoupIndex::isCrossedBySegment().
    bool crossesSurface(const Point3D& a, const Point3D& b) const;

private:
    TriangleSoupIndex triangles_;
};

} // namespace Meshing
