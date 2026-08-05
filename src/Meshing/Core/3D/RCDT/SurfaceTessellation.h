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
    /// Builds a UV-grid tessellation of surface's trimmed patch at a
    /// resolution whose triangle edge lengths are at most targetCellSize.
    /// For flat surfaces any resolution is exact, so a small fixed sample
    /// count is used regardless of targetCellSize. The computed sample count
    /// is capped at MAXIMUM_SAMPLES_PER_DIRECTION to bound memory and cost.
    /// A cell isn't clipped to the exact trim curve, so the tessellation can
    /// extend up to one grid cell past the true trimmed patch near its edge
    /// — harmless here, since callers (RestrictedTriangulation) separately
    /// check that the points they care about are within the true trim
    /// boundary; this tessellation only needs to not have gaps.
    void build(const Geometry3D::ISurface3D& surface, double targetCellSize);

    /// Whether segment (a, b) crosses this tessellation — exact, checking
    /// RobustPredicates3D::segmentCrossesTriangle against every triangle
    /// whose axis-aligned bounding box the segment's own bounding box
    /// overlaps (a triangle outside that can't possibly be crossed, so the
    /// exact -- much more expensive -- predicate only runs on candidates
    /// that survive this cheap prefilter).
    bool crossesSurface(const Point3D& a, const Point3D& b) const;

private:
    struct BoundedTriangle
    {
        std::array<Point3D, 3> vertices;
        Point3D boundsMin;
        Point3D boundsMax;
    };

    // Uniform spatial grid accelerating crossesSurface(): instead of scanning
    // every triangle for each query, the grid narrows the search to only those
    // triangles whose grid cell(s) overlap the query segment's bounding box.
    // Each BoundedTriangle is inserted into every grid cell whose 3D bounds
    // overlap the triangle's own bounds; crossesSurface iterates only the
    // cells touched by the query segment. For short dual edges (typical during
    // RCDT refinement) only a small fraction of cells are visited, giving an
    // O(n / cells_touched) speedup over the flat linear scan.
    struct AccelGrid
    {
        Point3D gridMin;
        Point3D gridMax;
        Point3D cellSize;
        size_t resolutionX = 0;
        size_t resolutionY = 0;
        size_t resolutionZ = 0;
        std::vector<std::vector<size_t>> cells; // indexed by cellIndex()

        bool isBuilt() const { return resolutionX > 0; }
        size_t cellIndex(size_t x, size_t y, size_t z) const
        {
            return x * resolutionY * resolutionZ + y * resolutionZ + z;
        }
    };

    void addTriangle(const Point3D& p0, const Point3D& p1, const Point3D& p2);
    void buildAccelGrid();

    std::vector<BoundedTriangle> triangles_;
    AccelGrid accelGrid_;
};

} // namespace Meshing
