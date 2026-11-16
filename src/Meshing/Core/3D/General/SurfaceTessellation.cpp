#include "Meshing/Core/3D/General/SurfaceTessellation.h"

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

// Every sample is shifted by a non-simple fraction of a cell. An evenly spaced grid lands on the
// round-number coordinates CAD geometry favours (a box face's centre, a hole's centre), and a
// segment crossing exactly at a grid vertex is rejected by segmentCrossesTriangle as touching
// rather than crossing (OPE-169).
//
// U and V get different jitter: with equal jitter every cell diagonal falls on u_local == v_local,
// and a point with u == v (e.g. the circumcenter of a right triangle with legs on the axes) would
// sit exactly on a diagonal edge.
constexpr double GRID_JITTER_U = 0.37;
constexpr double GRID_JITTER_V = 0.61;

// One parameter direction of the sampling grid.
struct SampledDirection
{
    double minimum = 0.0;
    double maximum = 0.0;
    bool periodic = false;
    size_t columns = 0; // distinct sample columns
    size_t cells = 0;   // grid cells between them
    double jitter = 0.0;

    double parameterAt(size_t column) const
    {
        // Periodic: columns spread evenly (plus jitter) around the whole period, never on
        // minimum/maximum, which is an arbitrary cut. Non-periodic: the first column lies a
        // fraction of a cell before minimum and the last past maximum, so the cells cover the
        // whole interval. Jittering inward left a band up to a cell wide along the minimum edge,
        // exactly where a crease sits.
        if (periodic)
        {
            return minimum +
                   (maximum - minimum) * (static_cast<double>(column) + jitter) / static_cast<double>(columns);
        }
        return minimum +
               (maximum - minimum) * (static_cast<double>(column) + jitter - 1.0) / static_cast<double>(columns - 2);
    }
};

// The sampled points of the whole UV grid and which of them lie within the surface's trim
// boundary, both in row-major (u column, v column) order.
struct SampleGrid
{
    SampledDirection u;
    SampledDirection v;
    std::vector<Point3D> points;
    std::vector<bool> withinTrim;

    size_t index(size_t uColumn, size_t vColumn) const { return uColumn * v.columns + vColumn; }
};

struct Periodicity
{
    bool inU = false;
    bool inV = false;
};

SampledDirection sampledDirection(double minimum, double maximum, bool periodic, size_t samplesPerDirection,
                                  double jitter);
double estimateDiameter(const Geometry3D::ISurface3D& surface, const Common::BoundingBox2D& bounds);
size_t samplesPerDirectionFor(double diameter, double targetCellSize);
Periodicity detectPeriodicity(const Geometry3D::ISurface3D& surface, const Common::BoundingBox2D& bounds,
                              double diameter);
SampleGrid sampleSurface(const Geometry3D::ISurface3D& surface, const SampledDirection& u, const SampledDirection& v);
std::vector<TriangleSoupIndex::Triangle> emitTriangles(const SampleGrid& grid);

} // namespace

void SurfaceTessellation::build(const Geometry3D::ISurface3D& surface, double targetCellSize)
{
    if (targetCellSize <= 0.0)
    {
        triangles_.build({}); // no tessellation at all, not even a previous one
        return;
    }

    auto bounds = surface.getParameterBounds();
    auto diameter = estimateDiameter(surface, bounds);
    auto samplesPerDirection = samplesPerDirectionFor(diameter, targetCellSize);
    auto periodicity = detectPeriodicity(surface, bounds, diameter);

    auto u = sampledDirection(bounds.getUMin(), bounds.getUMax(), periodicity.inU, samplesPerDirection, GRID_JITTER_U);
    auto v = sampledDirection(bounds.getVMin(), bounds.getVMax(), periodicity.inV, samplesPerDirection, GRID_JITTER_V);

    triangles_.build(emitTriangles(sampleSurface(surface, u, v)));
}

std::optional<Point3D> SurfaceTessellation::findCrossingNearest(const Point3D& segmentStart,
                                                                const Point3D& segmentEnd,
                                                                const Point3D& target) const
{
    return triangles_.findCrossingNearest(segmentStart, segmentEnd, target);
}

namespace
{

// A periodic direction is a closed loop (the last cell wraps back to column 0); a non-periodic one
// is an open strip with two extra columns just past each end.
SampledDirection sampledDirection(double minimum, double maximum, bool periodic, size_t samplesPerDirection,
                                  double jitter)
{
    SampledDirection direction;
    direction.minimum = minimum;
    direction.maximum = maximum;
    direction.periodic = periodic;
    direction.columns = periodic ? samplesPerDirection : samplesPerDirection + 2;
    direction.cells = periodic ? direction.columns : direction.columns - 1;
    direction.jitter = jitter;
    return direction;
}

// Rough characteristic size of the surface's parameter-bounds footprint (the same 4-corner
// diameter idea as computeSurfaceDiameter() in SurfaceProjector.cpp).
double estimateDiameter(const Geometry3D::ISurface3D& surface, const Common::BoundingBox2D& bounds)
{
    std::array<Point3D, 4> corners = {
        surface.getPoint(bounds.getUMin(), bounds.getVMin()), surface.getPoint(bounds.getUMax(), bounds.getVMin()),
        surface.getPoint(bounds.getUMin(), bounds.getVMax()), surface.getPoint(bounds.getUMax(), bounds.getVMax())};

    double maximumDistance = 0.0;
    for (int i = 0; i < 4; ++i)
    {
        for (int j = i + 1; j < 4; ++j)
        {
            maximumDistance = std::max(maximumDistance, (corners[i] - corners[j]).norm());
        }
    }

    return maximumDistance > 0.0 ? maximumDistance : 1.0;
}

// Scaled by targetCellSize for every surface shape, flat included: a flat surface's triangles are
// exact at any resolution, but the jittered grid's edge-coverage gap near the trim boundary is not.
// A coarse grid leaves a band along a crease that no triangle covers (SaddleSurfaceMesh: 0.10-0.14
// gaps against a 0.052 floor, producing hundreds of spurious "hole" edges).
size_t samplesPerDirectionFor(double diameter, double targetCellSize)
{
    // Bounds memory and cost, which are quadratic in this. Cells coarser than targetCellSize void
    // WeightedDualRestriction's guarantee that cells of minimumEdgeLength / 2 classify any face down
    // to the floor, so the cap warns when it binds (OPE-208).
    constexpr size_t MAXIMUM_SAMPLES_PER_DIRECTION = 400;

    auto computed = static_cast<size_t>(std::ceil(diameter / targetCellSize));
    if (computed > MAXIMUM_SAMPLES_PER_DIRECTION)
    {
        spdlog::warn("SurfaceTessellation: sample cap reached -- {} columns wanted for a target cell size of {}, "
                     "capped at {}, so cells are {} across and classification is no longer guaranteed at that floor",
                     computed, targetCellSize, MAXIMUM_SAMPLES_PER_DIRECTION,
                     diameter / static_cast<double>(MAXIMUM_SAMPLES_PER_DIRECTION));
    }
    return std::clamp(computed, size_t{2}, MAXIMUM_SAMPLES_PER_DIRECTION);
}

// ISurface3D has no periodicity query, so detect it: a periodic direction's two parameter extremes
// map to the same point (probed at the other direction's midpoint, to avoid corner and pole
// degeneracy). Without it a plain grid leaves a gap along a periodic surface's seam (OPE-171).
Periodicity detectPeriodicity(const Geometry3D::ISurface3D& surface, const Common::BoundingBox2D& bounds,
                              double diameter)
{
    constexpr double PERIODICITY_RELATIVE_TOLERANCE = 1e-6;
    double periodicityTolerance = PERIODICITY_RELATIVE_TOLERANCE * diameter;

    double uMin = bounds.getUMin();
    double uMax = bounds.getUMax();
    double vMin = bounds.getVMin();
    double vMax = bounds.getVMax();
    double midU = 0.5 * (uMin + uMax);
    double midV = 0.5 * (vMin + vMax);

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
        double uParameter = u.parameterAt(uColumn);
        for (size_t vColumn = 0; vColumn < v.columns; ++vColumn)
        {
            double vParameter = v.parameterAt(vColumn);
            grid.points[grid.index(uColumn, vColumn)] = surface.getPoint(uParameter, vParameter);
            grid.withinTrim[grid.index(uColumn, vColumn)] = surface.isUVWithinTrimmedBoundary(uParameter, vParameter);
        }
    }
    return grid;
}

// Two triangles per cell, split along the corner00--corner11 diagonal. The last cell of a periodic
// direction wraps back onto column 0, leaving no gap at the seam.
//
// A cell is kept if ANY corner is within the trim: requiring all four shrinks the tessellation
// inward by up to a cell and opens gaps near ordinary edges. Over-including is safe, because
// WeightedDualRestriction accepts a crossing only within the true trimmed patch.
std::vector<TriangleSoupIndex::Triangle> emitTriangles(const SampleGrid& grid)
{
    std::vector<TriangleSoupIndex::Triangle> triangles;

    for (size_t uCell = 0; uCell < grid.u.cells; ++uCell)
    {
        auto uNext = (uCell + 1) % grid.u.columns;
        for (size_t vCell = 0; vCell < grid.v.cells; ++vCell)
        {
            auto vNext = (vCell + 1) % grid.v.columns;

            auto corner00 = grid.index(uCell, vCell);
            auto corner10 = grid.index(uNext, vCell);
            auto corner01 = grid.index(uCell, vNext);
            auto corner11 = grid.index(uNext, vNext);

            bool anyCornerWithinTrim = grid.withinTrim[corner00] || grid.withinTrim[corner10] ||
                                       grid.withinTrim[corner01] || grid.withinTrim[corner11];
            if (!anyCornerWithinTrim) continue;

            triangles.push_back({grid.points[corner00], grid.points[corner10], grid.points[corner11]});
            triangles.push_back({grid.points[corner00], grid.points[corner11], grid.points[corner01]});
        }
    }

    return triangles;
}

} // namespace

} // namespace Meshing
