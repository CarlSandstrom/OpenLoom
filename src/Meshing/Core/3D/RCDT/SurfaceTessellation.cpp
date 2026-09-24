#include "Meshing/Core/3D/RCDT/SurfaceTessellation.h"

#include "Common/BoundingBox2D.h"
#include "Geometry/3D/Base/ISurface3D.h"

#include "spdlog/spdlog.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

namespace Meshing
{

namespace
{

// Fraction of a cell width to shift every sample by. A plain evenly-spaced
// grid lands exactly on round-number coordinates -- a box face's center,
// a hole's exact center, anything at a simple fraction of the surface's
// extent -- and CAD geometry (and test fixtures) both favor exactly those
// numbers. When a segment's true crossing point coincides exactly with a
// tessellation grid vertex, segmentCrossesTriangle correctly (by design)
// treats that as touching a vertex, not crossing the interior, and
// rejects it -- a false negative for a case that's actually extremely
// common, not a rare edge case (see OPE-169). Shifting the whole grid by
// a non-simple fraction of a cell makes that coincidence very unlikely
// without needing to special-case any particular geometry.
//
// U and V need *different* jitter, not just any jitter: each grid cell
// is split into two triangles along the diagonal from its (i,j) corner
// to its (i+1,j+1) corner, so with equal U/V jitter and equal sample
// counts, every diagonal falls exactly on the line u_local == v_local
// within its cell -- and a point with u == v (e.g. the circumcenter of
// any right triangle whose legs sit on the U and V axes -- an extremely
// ordinary case, not a contrived one) then sits exactly on that
// diagonal edge instead of a triangle's interior. Different jitter per
// axis breaks that alignment too.
constexpr double GRID_JITTER_U = 0.37;
constexpr double GRID_JITTER_V = 0.61;

// One parameter direction of the sampling grid: the parameter interval, how
// many sample columns it carries, and how they are spaced.
struct SampledDirection
{
    double minimum = 0.0;
    double maximum = 0.0;
    bool periodic = false;
    size_t columns = 0; // distinct sample columns
    size_t cells = 0;   // grid cells between them
    double jitter = 0.0;

    /// The parameter value sampled by the given column.
    double parameterAt(size_t column) const
    {
        // Periodic: columns points spread evenly (plus jitter) around the
        // *whole* period, never touching minimum/maximum themselves -- there's
        // no reason to privilege the arbitrary parametric cut point.
        // Non-periodic: columns points spanning [minimum, maximum], jittered
        // inward.
        const double spacings =
            periodic ? static_cast<double>(columns) : static_cast<double>(columns - 1) + jitter;
        return minimum + (maximum - minimum) * (static_cast<double>(column) + jitter) / spacings;
    }
};

// A periodic direction is sampled as a closed loop (samplesPerDirection
// distinct columns, the last cell wrapping back to the first column) rather
// than an open strip (samplesPerDirection + 1 columns, both endpoints sampled
// once each, so one fewer cell than columns) -- there's no real "boundary" to
// place two separate columns on.
SampledDirection sampledDirection(double minimum, double maximum, bool periodic, size_t samplesPerDirection,
                                  double jitter)
{
    SampledDirection direction;
    direction.minimum = minimum;
    direction.maximum = maximum;
    direction.periodic = periodic;
    direction.columns = periodic ? samplesPerDirection : samplesPerDirection + 1;
    direction.cells = periodic ? direction.columns : direction.columns - 1;
    direction.jitter = jitter;
    return direction;
}

// The sampled points of the whole UV grid and which of them lie within the
// surface's trim boundary, both in row-major (u column, v column) order.
struct SampleGrid
{
    SampledDirection u;
    SampledDirection v;
    std::vector<Point3D> points;
    std::vector<bool> withinTrim;

    size_t index(size_t uColumn, size_t vColumn) const { return uColumn * v.columns + vColumn; }
};

// Rough characteristic size of the surface's parameter-bounds footprint.
// Reuses the same 4-corner-diameter idea as computeSurfaceDiameter() in SurfaceProjector.cpp.
double estimateDiameter(const Geometry3D::ISurface3D& surface, const Common::BoundingBox2D& bounds)
{
    const std::array<Point3D, 4> corners = {
        surface.getPoint(bounds.getUMin(), bounds.getVMin()), surface.getPoint(bounds.getUMax(), bounds.getVMin()),
        surface.getPoint(bounds.getUMin(), bounds.getVMax()), surface.getPoint(bounds.getUMax(), bounds.getVMax())};

    double maximumDistance = 0.0;
    for (int i = 0; i < 4; ++i)
        for (int j = i + 1; j < 4; ++j)
            maximumDistance = std::max(maximumDistance, (corners[i] - corners[j]).norm());

    return maximumDistance > 0.0 ? maximumDistance : 1.0;
}

// Sample columns per direction for a surface of this diameter.
//
// Scaled against targetCellSize regardless of whether the surface is
// flat: a flat surface's *triangles* are exact at any resolution, but
// the jittered grid's own edge-coverage gap (~0.6*extent/samples, see the
// jitter comment above) is not -- it can leave a band near the surface's
// trim boundary, exactly where a crease with a neighboring surface sits,
// that no triangle covers. A fixed low sample count for flat surfaces
// used to leave that gap far wider than minimumEdgeLength_ once
// refinement pushed elements down to the floor (confirmed on the
// SaddleSurfaceMesh stress test: ~0.10-0.14 unit gaps against a 0.052
// floor, producing hundreds of spurious non-manifold "hole" edges right
// along creases). Coverage, not accuracy, is what this resolution buys,
// so it must scale the same way for every surface shape.
size_t samplesPerDirectionFor(double diameter, double targetCellSize)
{
    // Upper bound, bounding worst-case tessellation memory and cost for very
    // large or very fine-resolution meshes. Cost is quadratic in this, so it
    // cannot simply be raised.
    //
    // It reports when it binds (OPE-208). Everything above this point is
    // tessellated COARSER than targetCellSize asked for, which voids the
    // guarantee the cell size exists to provide -- see
    // TESSELLATION_CELL_SIZE_FACTOR in DualEdgeRestrictionOracle.cpp, whose
    // claim that cells below minimumEdgeLength / 2 can classify any face down
    // to that floor holds only while this clamp is inactive. Clamping
    // silently left no way to tell from the outside that it had stopped
    // holding, which is the whole failure mode of a cost cap standing in for
    // a correctness parameter.
    constexpr size_t MAXIMUM_SAMPLES_PER_DIRECTION = 400;

    const size_t computed = static_cast<size_t>(std::ceil(diameter / targetCellSize));
    if (computed > MAXIMUM_SAMPLES_PER_DIRECTION)
    {
        spdlog::warn("SurfaceTessellation: sample cap reached -- {} columns wanted for a target cell size of {}, "
                     "capped at {}, so cells are {} across and classification is no longer guaranteed at that floor",
                     computed, targetCellSize, MAXIMUM_SAMPLES_PER_DIRECTION,
                     diameter / static_cast<double>(MAXIMUM_SAMPLES_PER_DIRECTION));
    }
    return std::clamp(computed, size_t{2}, MAXIMUM_SAMPLES_PER_DIRECTION);
}

// Whether the surface wraps in each parameter direction.
struct Periodicity
{
    bool inU = false;
    bool inV = false;
};

// ISurface3D has no explicit periodicity query, so detect it numerically:
// a periodic direction's two parameter extremes map to the same physical
// point (probed at the other direction's midpoint, to sidestep any
// corner/pole degeneracy). Works uniformly for any backend, not just OCC.
// This matters because a plain non-wrapping grid leaves a real gap right
// along the seam of a periodic surface (e.g. a torus's major-circle seam,
// OPE-171): nothing samples exactly the boundary column *and* its twin at
// the other end of the period, so no triangle ever covers the strip
// between the last sampled column and the first.
Periodicity detectPeriodicity(const Geometry3D::ISurface3D& surface, const Common::BoundingBox2D& bounds,
                              double diameter)
{
    constexpr double PERIODICITY_RELATIVE_TOLERANCE = 1e-6;
    const double periodicityTolerance = PERIODICITY_RELATIVE_TOLERANCE * diameter;

    const double uMin = bounds.getUMin();
    const double uMax = bounds.getUMax();
    const double vMin = bounds.getVMin();
    const double vMax = bounds.getVMax();
    const double midV = 0.5 * (vMin + vMax);
    const double midU = 0.5 * (uMin + uMax);

    Periodicity periodicity;
    periodicity.inU = (surface.getPoint(uMin, midV) - surface.getPoint(uMax, midV)).norm() < periodicityTolerance;
    periodicity.inV = (surface.getPoint(midU, vMin) - surface.getPoint(midU, vMax)).norm() < periodicityTolerance;
    return periodicity;
}

SampleGrid sampleSurface(const Geometry3D::ISurface3D& surface, const SampledDirection& u, const SampledDirection& v)
{
    SampleGrid grid;
    grid.u = u;
    grid.v = v;
    grid.points.resize(u.columns * v.columns);
    grid.withinTrim.resize(u.columns * v.columns);

    for (size_t uColumn = 0; uColumn < u.columns; ++uColumn)
    {
        const double uParameter = u.parameterAt(uColumn);
        for (size_t vColumn = 0; vColumn < v.columns; ++vColumn)
        {
            const double vParameter = v.parameterAt(vColumn);
            grid.points[grid.index(uColumn, vColumn)] = surface.getPoint(uParameter, vParameter);
            grid.withinTrim[grid.index(uColumn, vColumn)] =
                surface.isUVWithinTrimmedBoundary(uParameter, vParameter);
        }
    }
    return grid;
}

// Two triangles per grid cell, split along the cell's corner00--corner11
// diagonal. A cell whose corner columns wrap (the last cell of a periodic
// direction) closes the loop back onto column 0, leaving no gap at the seam.
std::vector<TriangleSoupIndex::Triangle> emitTriangles(const SampleGrid& grid)
{
    std::vector<TriangleSoupIndex::Triangle> triangles;

    for (size_t uCell = 0; uCell < grid.u.cells; ++uCell)
    {
        const size_t uNext = (uCell + 1) % grid.u.columns;
        for (size_t vCell = 0; vCell < grid.v.cells; ++vCell)
        {
            const size_t vNext = (vCell + 1) % grid.v.columns;

            const size_t corner00 = grid.index(uCell, vCell);
            const size_t corner10 = grid.index(uNext, vCell);
            const size_t corner01 = grid.index(uCell, vNext);
            const size_t corner11 = grid.index(uNext, vNext);

            // Include the cell if ANY corner is within the trim, not only
            // when all 4 are: requiring all 4 shrinks the tessellation
            // inward from the true trim boundary by up to one grid cell
            // width, which is large enough (at this resolution) to open
            // gaps near ordinary edges, not just the ones this class exists
            // to handle. Over-including here is safe -- classifyFace()
            // separately gates on the face's own vertices actually lying
            // within the true trim boundary (verticesWithinTrimmedBoundary),
            // so a tessellation triangle that pokes slightly past the real
            // edge never causes a face to be accepted that shouldn't be.
            if (!grid.withinTrim[corner00] && !grid.withinTrim[corner10] && !grid.withinTrim[corner01] &&
                !grid.withinTrim[corner11])
                continue;

            triangles.push_back({grid.points[corner00], grid.points[corner10], grid.points[corner11]});
            triangles.push_back({grid.points[corner00], grid.points[corner11], grid.points[corner01]});
        }
    }

    return triangles;
}

} // namespace

void SurfaceTessellation::build(const Geometry3D::ISurface3D& surface, double targetCellSize)
{
    if (targetCellSize <= 0.0)
    {
        triangles_.build({}); // no tessellation at all, not even a previous one
        return;
    }

    const Common::BoundingBox2D bounds = surface.getParameterBounds();
    const double diameter = estimateDiameter(surface, bounds);
    const size_t samplesPerDirection = samplesPerDirectionFor(diameter, targetCellSize);
    const Periodicity periodicity = detectPeriodicity(surface, bounds, diameter);

    const SampledDirection u =
        sampledDirection(bounds.getUMin(), bounds.getUMax(), periodicity.inU, samplesPerDirection, GRID_JITTER_U);
    const SampledDirection v =
        sampledDirection(bounds.getVMin(), bounds.getVMax(), periodicity.inV, samplesPerDirection, GRID_JITTER_V);

    triangles_.build(emitTriangles(sampleSurface(surface, u, v)));
}

bool SurfaceTessellation::crossesSurface(const Point3D& a, const Point3D& b) const
{
    return triangles_.isCrossedBySegment(a, b);
}

} // namespace Meshing
