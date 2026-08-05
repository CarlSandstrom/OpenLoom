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
    /// RobustPredicates3D::segmentCrossesTriangle against every triangle
    /// whose axis-aligned bounding box the segment's own bounding box
    /// overlaps (a triangle outside that can't possibly be crossed, so the
    /// exact -- much more expensive -- predicate only runs on candidates
    /// that survive this cheap prefilter).
    bool crossesSurface(const Point3D& a, const Point3D& b) const;

    /// Rebuilds this tessellation at a finer resolution if its current cell
    /// size isn't small relative to requiredEdgeLength. No-op if build()
    /// hasn't been called yet, if the current resolution is already fine
    /// enough, or if segment (a, b) is implausibly long relative to the
    /// surface's own size (see the .cpp for why -- RestrictedTriangulation's
    /// bounding-supertet-node fallback, not a real local dual edge).
    ///
    /// For a planar surface any tessellation resolution represents the exact
    /// surface, so cell size never matters. For a curved surface, the
    /// tessellation is only a chordal approximation: once real mesh elements
    /// being tested against it (via crossesSurface) shrink below the
    /// tessellation's own fixed cell size, crossesSurface's answers can
    /// become inconsistent with the true continuous surface. This keeps the
    /// oracle's resolution tracking the finest element it's actually being
    /// asked about, rebuilding (full recompute, not local subdivision) only
    /// when needed. A full rebuild -- not local cell subdivision, tried and
    /// reverted -- because Priority 4's repair loop implicitly relies on any
    /// one resolution upgrade fixing the *whole* surface's classifications
    /// at once (self-healing via later reclassification only revisits faces
    /// near a freshly inserted node, so a purely local fix leaves other,
    /// unrelated stale-classified faces elsewhere on the surface to never
    /// get revisited -- confirmed empirically: local refinement's defect
    /// count grew without bound instead of converging).
    void ensureResolution(const Point3D& a, const Point3D& b, double requiredEdgeLength);

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

    // Remembered so ensureResolution() can rebuild at a different sample
    // count without the caller needing to keep the surface reference around.
    const Geometry3D::ISurface3D* surface_ = nullptr;
    size_t samplesPerDirection_ = 0;

    // Maximum edge length among all triangles produced by the most recent
    // build() -- the worst-case (largest, i.e. coarsest) cell, so
    // ensureResolution() triggers conservatively whenever any cell could be
    // too coarse for what's being tested. Zero if triangles_ is empty.
    double characteristicCellSize_ = 0.0;

    // Whether surface's normal is numerically constant across its parameter
    // bounds -- a true plane, for which any tessellation resolution is
    // exact. Computed in build(); makes ensureResolution() a no-op.
    bool isFlat_ = false;

    // Rough physical size of the surface, computed in build(). A query
    // segment much longer than this isn't a real local dual edge -- it's
    // RestrictedTriangulation's bounding-supertet-node fallback (see
    // computeDualEdgeEndpoint()), whose far endpoint can be hundreds of
    // units away. ensureResolution() uses this to recognize and skip that
    // case, rather than let it drive a rebuild sized for a segment that
    // doesn't reflect real local mesh density.
    double surfaceDiameter_ = 0.0;
};

} // namespace Meshing
