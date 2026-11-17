#pragma once

#include "Common/Types.h"
#include "Meshing/Core/3D/General/TriangleSoupIndex.h"

#include <array>
#include <optional>
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
 * WeightedDualRestriction needs to know, robustly, whether a segment crosses a
 * surface. Testing that against the surface's own continuous geometry needs a
 * near-tangent tolerance, which can misclassify a segment whose endpoints are
 * only fractions of a unit from the surface (OPE-169). Asking instead whether
 * the segment crosses any triangle of a fixed tessellation turns it into a
 * point-based predicate (RobustPredicates3D::segmentCrossesTriangle),
 * evaluated exactly however close to degenerate the input is.
 *
 * Built from the ISurface3D interface alone (getPoint(), getParameterBounds(),
 * isUVWithinTrimmedBoundary()), so it works the same for a plane or a NURBS
 * patch and does not tie the mesher to OpenCascade.
 *
 * This class is the UV sampling half of that job: where to place samples and
 * which cells to emit. The triangles are held in a TriangleSoupIndex, which
 * knows nothing about surfaces and answers the crossing query.
 */
class SurfaceTessellation
{
public:
    /// Builds a UV-grid tessellation of surface's trimmed patch with triangle
    /// edges at most targetCellSize long, for flat surfaces too: the jittered
    /// grid's coverage gap near the trim boundary needs the resolution even
    /// where the triangles themselves are exact. The sample count per
    /// direction is capped to bound cost, and the cap warns when it binds.
    ///
    /// A cell is not clipped to the trim curve, so the tessellation can extend
    /// up to one cell past the trimmed patch. That is harmless: the caller
    /// checks that an accepted crossing lies within the true trim boundary.
    void build(const Geometry3D::ISurface3D& surface, double targetCellSize);

    /// A crossing of the tessellation and the surface parameters there,
    /// interpolated linearly across the crossed triangle. Across a periodic
    /// seam the parameters are unwrapped, so they may lie up to a cell past
    /// the parameter bounds.
    struct Crossing
    {
        Point3D point;
        Point2D uv;
    };

    /// Where the segment crosses this tessellation, nearest to target; see
    /// TriangleSoupIndex::findCrossingNearest(). The point lies on the
    /// tessellation, not exactly on the surface.
    std::optional<Crossing> findCrossingNearest(const Point3D& segmentStart,
                                                const Point3D& segmentEnd,
                                                const Point3D& target) const;

private:
    TriangleSoupIndex triangles_;
    std::vector<std::array<Point2D, 3>> triangleParameters_; // indexed like the soup's triangles
};

} // namespace Meshing
