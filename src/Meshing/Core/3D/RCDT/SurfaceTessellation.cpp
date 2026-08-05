#include "Meshing/Core/3D/RCDT/SurfaceTessellation.h"

#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Core/3D/General/RobustPredicates3D.h"

#include <algorithm>
#include <cmath>
#include <optional>

namespace Meshing
{

namespace
{

// Upper bound on samplesPerDirection, bounding worst-case tessellation
// memory and cost for very large or very fine-resolution meshes.
constexpr size_t MAXIMUM_SAMPLES_PER_DIRECTION = 400;

// Sample count used for flat surfaces. Any resolution is exact for a plane,
// so accuracy isn't the constraint -- but the jittered grid must still cover
// the trimmed patch without gaps near its edges. With V-jitter of 0.61 and N
// samples over range L, the edge coverage gap is ~0.61*L/N; at N=24 this is
// ~2.5% of the surface's extent in each direction, which keeps the gap well
// below typical mesh element sizes in practice.
constexpr size_t FLAT_SURFACE_SAMPLES_PER_DIRECTION = 24;

// Whether two axis-aligned boxes [aMin, aMax] and [bMin, bMax] overlap in all
// 3 axes. A necessary (not sufficient) condition for the shapes they bound to
// actually intersect -- see crossesSurface().
bool boundsOverlap(const Point3D& aMin, const Point3D& aMax, const Point3D& bMin, const Point3D& bMax)
{
    return aMin.x() <= bMax.x() && bMin.x() <= aMax.x() && aMin.y() <= bMax.y() && bMin.y() <= aMax.y() &&
           aMin.z() <= bMax.z() && bMin.z() <= aMax.z();
}

// Rough characteristic size of the surface's parameter-bounds footprint.
// Reuses the same 4-corner-diameter idea as SurfaceProjector::computeSurfaceDiameter.
double estimateDiameter(const Geometry3D::ISurface3D& surface, double uMin, double uMax, double vMin, double vMax)
{
    const std::array<Point3D, 4> corners = {surface.getPoint(uMin, vMin), surface.getPoint(uMax, vMin),
                                            surface.getPoint(uMin, vMax), surface.getPoint(uMax, vMax)};

    double maximumDistance = 0.0;
    for (int i = 0; i < 4; ++i)
        for (int j = i + 1; j < 4; ++j)
            maximumDistance = std::max(maximumDistance, (corners[i] - corners[j]).norm());

    return maximumDistance > 0.0 ? maximumDistance : 1.0;
}

// Whether surface's normal is (numerically) identical across a jittered
// grid of probe points spanning its parameter bounds. For a genuinely flat
// surface a tessellation represents the surface exactly no matter how
// coarse it is -- a small fixed sample count suffices and we never need to
// size it against targetCellSize. Any real curvature produces normal
// variation many orders of magnitude above the tolerance here, even over a
// tiny patch -- as long as a probe lands where the slope is actually
// nonzero. A localized, symmetric feature (e.g. a bump) has zero slope
// exactly at its own center by construction, so corners/center alone can be
// fooled into reading it as flat; jittering the grid makes landing exactly
// on such a point very unlikely without relying on it being impossible.
bool isEffectivelyFlat(const Geometry3D::ISurface3D& surface, double uMin, double uMax, double vMin, double vMax)
{
    constexpr double FLATNESS_DOT_THRESHOLD = 1.0 - 1e-9;
    constexpr size_t FLATNESS_PROBE_DIVISIONS = 8;
    constexpr double FLATNESS_PROBE_JITTER_U = 0.29;
    constexpr double FLATNESS_PROBE_JITTER_V = 0.71;

    std::optional<Point3D> referenceNormal;
    for (size_t i = 0; i < FLATNESS_PROBE_DIVISIONS; ++i)
    {
        const double u = uMin + (uMax - uMin) * (static_cast<double>(i) + FLATNESS_PROBE_JITTER_U) /
                                    static_cast<double>(FLATNESS_PROBE_DIVISIONS);
        for (size_t j = 0; j < FLATNESS_PROBE_DIVISIONS; ++j)
        {
            const double v = vMin + (vMax - vMin) * (static_cast<double>(j) + FLATNESS_PROBE_JITTER_V) /
                                        static_cast<double>(FLATNESS_PROBE_DIVISIONS);

            const auto raw = surface.getNormal(u, v);
            const Point3D normal(raw[0], raw[1], raw[2]);
            const double length = normal.norm();
            if (length <= 0.0)
                return false;
            const Point3D unitNormal = normal / length;

            if (!referenceNormal)
            {
                referenceNormal = unitNormal;
                continue;
            }
            if (referenceNormal->dot(unitNormal) < FLATNESS_DOT_THRESHOLD)
                return false;
        }
    }
    return true;
}

} // namespace

void SurfaceTessellation::build(const Geometry3D::ISurface3D& surface, double targetCellSize)
{
    triangles_.clear();
    accelGrid_ = {};
    if (targetCellSize <= 0.0)
        return;

    const auto bounds = surface.getParameterBounds();
    const double uMin = bounds.getUMin();
    const double uMax = bounds.getUMax();
    const double vMin = bounds.getVMin();
    const double vMax = bounds.getVMax();

    const double diameter = estimateDiameter(surface, uMin, uMax, vMin, vMax);

    // Flat surfaces are exact at any resolution -- use a small fixed count
    // rather than scaling against targetCellSize (which could be very small,
    // producing a needlessly large tessellation of a surface where it makes
    // no difference).
    size_t samplesPerDirection;
    if (isEffectivelyFlat(surface, uMin, uMax, vMin, vMax))
    {
        samplesPerDirection = FLAT_SURFACE_SAMPLES_PER_DIRECTION;
    }
    else
    {
        const size_t computed = static_cast<size_t>(std::ceil(diameter / targetCellSize));
        samplesPerDirection = std::clamp(computed, size_t{2}, MAXIMUM_SAMPLES_PER_DIRECTION);
    }

    // ISurface3D has no explicit periodicity query, so detect it numerically:
    // a periodic direction's two parameter extremes map to the same physical
    // point (probed at the other direction's midpoint, to sidestep any
    // corner/pole degeneracy). Works uniformly for any backend, not just OCC.
    // This matters because a plain non-wrapping grid leaves a real gap right
    // along the seam of a periodic surface (e.g. a torus's major-circle seam,
    // OPE-171): nothing samples exactly the boundary column *and* its twin at
    // the other end of the period, so no triangle ever covers the strip
    // between the last sampled column and the first.
    constexpr double PERIODICITY_RELATIVE_TOLERANCE = 1e-6;
    const double periodicityTolerance = PERIODICITY_RELATIVE_TOLERANCE * diameter;

    const double midV = 0.5 * (vMin + vMax);
    const double midU = 0.5 * (uMin + uMax);
    const bool periodicU = (surface.getPoint(uMin, midV) - surface.getPoint(uMax, midV)).norm() < periodicityTolerance;
    const bool periodicV = (surface.getPoint(midU, vMin) - surface.getPoint(midU, vMax)).norm() < periodicityTolerance;

    // A periodic direction is sampled as a closed loop (samplesPerDirection
    // distinct columns, last one wrapping back to the first) rather than an
    // open strip (samplesPerDirection + 1 columns, both endpoints sampled
    // once each) -- there's no real "boundary" to place two separate columns
    // on.
    const size_t uColumns = periodicU ? samplesPerDirection : samplesPerDirection + 1;
    const size_t vColumns = periodicV ? samplesPerDirection : samplesPerDirection + 1;

    const auto index = [vColumns](size_t i, size_t j)
    {
        return i * vColumns + j;
    };

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

    std::vector<Point3D> gridPoints(uColumns * vColumns);
    std::vector<bool> withinTrim(uColumns * vColumns);

    for (size_t i = 0; i < uColumns; ++i)
    {
        // Periodic: samplesPerDirection points spread evenly (plus jitter)
        // around the *whole* period, never touching uMin/uMax themselves --
        // there's no reason to privilege the arbitrary parametric cut point.
        // Non-periodic: gridSize = samplesPerDirection + 1 points spanning
        // [uMin, uMax], jittered inward, same as before.
        const double u = periodicU ? uMin + (uMax - uMin) * (static_cast<double>(i) + GRID_JITTER_U) /
                                                  static_cast<double>(samplesPerDirection)
                                    : uMin + (uMax - uMin) * (static_cast<double>(i) + GRID_JITTER_U) /
                                                  (static_cast<double>(uColumns - 1) + GRID_JITTER_U);
        for (size_t j = 0; j < vColumns; ++j)
        {
            const double v = periodicV ? vMin + (vMax - vMin) * (static_cast<double>(j) + GRID_JITTER_V) /
                                                      static_cast<double>(samplesPerDirection)
                                        : vMin + (vMax - vMin) * (static_cast<double>(j) + GRID_JITTER_V) /
                                                      (static_cast<double>(vColumns - 1) + GRID_JITTER_V);
            const Point3D point = surface.getPoint(u, v);
            gridPoints[index(i, j)] = point;
            withinTrim[index(i, j)] = surface.isUVWithinTrimmedBoundary(u, v);
        }
    }

    // Non-periodic: uColumns - 1 cells between uColumns distinct columns.
    // Periodic: uColumns cells -- the last one wraps from column uColumns-1
    // back to column 0, closing the loop with no gap.
    const size_t uCells = periodicU ? uColumns : uColumns - 1;
    const size_t vCells = periodicV ? vColumns : vColumns - 1;

    for (size_t i = 0; i < uCells; ++i)
    {
        const size_t iNext = (i + 1) % uColumns;
        for (size_t j = 0; j < vCells; ++j)
        {
            const size_t jNext = (j + 1) % vColumns;

            const size_t i00 = index(i, j);
            const size_t i10 = index(iNext, j);
            const size_t i01 = index(i, jNext);
            const size_t i11 = index(iNext, jNext);

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
            if (!withinTrim[i00] && !withinTrim[i10] && !withinTrim[i01] && !withinTrim[i11])
                continue;

            addTriangle(gridPoints[i00], gridPoints[i10], gridPoints[i11]);
            addTriangle(gridPoints[i00], gridPoints[i11], gridPoints[i01]);
        }
    }

    buildAccelGrid();
}

void SurfaceTessellation::addTriangle(const Point3D& p0, const Point3D& p1, const Point3D& p2)
{
    const Point3D boundsMin = p0.cwiseMin(p1).cwiseMin(p2);
    const Point3D boundsMax = p0.cwiseMax(p1).cwiseMax(p2);
    triangles_.push_back({{p0, p1, p2}, boundsMin, boundsMax});
}

void SurfaceTessellation::buildAccelGrid()
{
    accelGrid_ = {};
    if (triangles_.empty())
        return;

    // Compute the 3D bounding box of all tessellation triangles.
    Point3D gridMin = triangles_[0].boundsMin;
    Point3D gridMax = triangles_[0].boundsMax;
    for (const auto& triangle : triangles_)
    {
        gridMin = gridMin.cwiseMin(triangle.boundsMin);
        gridMax = gridMax.cwiseMax(triangle.boundsMax);
    }

    // Expand slightly so that triangles exactly on the boundary fall inside a
    // cell rather than rounding to an out-of-range index.
    constexpr double GRID_EPSILON = 1e-10;
    gridMin -= Point3D::Constant(GRID_EPSILON);
    gridMax += Point3D::Constant(GRID_EPSILON);

    // Choose the grid resolution so each cell holds roughly 1–4 triangles on
    // average. The tessellation lies on a 2D surface, not a 3D volume, so
    // many cells are empty -- that's fine; crossesSurface only visits cells
    // overlapping the query segment's bounding box, not all cells.
    constexpr size_t MINIMUM_GRID_RESOLUTION = 4;
    constexpr size_t MAXIMUM_GRID_RESOLUTION = 50;
    const size_t resolution = std::clamp(
        static_cast<size_t>(std::cbrt(static_cast<double>(triangles_.size()))),
        MINIMUM_GRID_RESOLUTION, MAXIMUM_GRID_RESOLUTION);

    accelGrid_.gridMin = gridMin;
    accelGrid_.gridMax = gridMax;
    accelGrid_.resolutionX = resolution;
    accelGrid_.resolutionY = resolution;
    accelGrid_.resolutionZ = resolution;

    const Point3D range = gridMax - gridMin;
    // Floor each component at a small positive value to avoid division by zero
    // for degenerate surfaces that are flat in one coordinate direction.
    constexpr double MINIMUM_RANGE = 1e-12;
    accelGrid_.cellSize = Point3D(
        std::max(range.x() / static_cast<double>(resolution), MINIMUM_RANGE),
        std::max(range.y() / static_cast<double>(resolution), MINIMUM_RANGE),
        std::max(range.z() / static_cast<double>(resolution), MINIMUM_RANGE));

    accelGrid_.cells.resize(resolution * resolution * resolution);

    // Helper: clamp a continuous coordinate to a valid grid index.
    const auto toIndex = [&](double coordinate, double gridMinCoord, double cellSizeCoord) -> size_t
    {
        const double normalized = (coordinate - gridMinCoord) / cellSizeCoord;
        const long long index = static_cast<long long>(std::floor(normalized));
        return static_cast<size_t>(std::clamp(index, 0LL, static_cast<long long>(resolution) - 1LL));
    };

    for (size_t triangleIndex = 0; triangleIndex < triangles_.size(); ++triangleIndex)
    {
        const auto& triangle = triangles_[triangleIndex];

        const size_t xMin = toIndex(triangle.boundsMin.x(), gridMin.x(), accelGrid_.cellSize.x());
        const size_t yMin = toIndex(triangle.boundsMin.y(), gridMin.y(), accelGrid_.cellSize.y());
        const size_t zMin = toIndex(triangle.boundsMin.z(), gridMin.z(), accelGrid_.cellSize.z());
        const size_t xMax = toIndex(triangle.boundsMax.x(), gridMin.x(), accelGrid_.cellSize.x());
        const size_t yMax = toIndex(triangle.boundsMax.y(), gridMin.y(), accelGrid_.cellSize.y());
        const size_t zMax = toIndex(triangle.boundsMax.z(), gridMin.z(), accelGrid_.cellSize.z());

        for (size_t x = xMin; x <= xMax; ++x)
            for (size_t y = yMin; y <= yMax; ++y)
                for (size_t z = zMin; z <= zMax; ++z)
                    accelGrid_.cells[accelGrid_.cellIndex(x, y, z)].push_back(triangleIndex);
    }
}

bool SurfaceTessellation::crossesSurface(const Point3D& a, const Point3D& b) const
{
    const Point3D segmentMin = a.cwiseMin(b);
    const Point3D segmentMax = a.cwiseMax(b);

    if (!accelGrid_.isBuilt())
    {
        for (const auto& triangle : triangles_)
        {
            if (!boundsOverlap(segmentMin, segmentMax, triangle.boundsMin, triangle.boundsMax))
                continue;
            if (RobustPredicates3D::segmentCrossesTriangle(a, b, triangle.vertices[0], triangle.vertices[1],
                                                            triangle.vertices[2]))
                return true;
        }
        return false;
    }

    // Spatial grid acceleration: only visit grid cells whose 3D bounds overlap
    // the segment's bounding box, then test only the triangles in those cells.
    // A triangle may appear in more than one cell if its bounding box spans a
    // cell boundary; the duplicate test is harmless (at worst a redundant true
    // return that terminates the loop early, or a redundant false that just
    // wastes a little work).
    const auto& grid = accelGrid_;
    const auto toIndex = [&](double coordinate, double gridMinCoord, double cellSizeCoord,
                             size_t maxIndex) -> size_t
    {
        const double normalized = (coordinate - gridMinCoord) / cellSizeCoord;
        const long long index = static_cast<long long>(std::floor(normalized));
        return static_cast<size_t>(
            std::clamp(index, 0LL, static_cast<long long>(maxIndex) - 1LL));
    };

    const size_t xMin = toIndex(segmentMin.x(), grid.gridMin.x(), grid.cellSize.x(), grid.resolutionX);
    const size_t yMin = toIndex(segmentMin.y(), grid.gridMin.y(), grid.cellSize.y(), grid.resolutionY);
    const size_t zMin = toIndex(segmentMin.z(), grid.gridMin.z(), grid.cellSize.z(), grid.resolutionZ);
    const size_t xMax = toIndex(segmentMax.x(), grid.gridMin.x(), grid.cellSize.x(), grid.resolutionX);
    const size_t yMax = toIndex(segmentMax.y(), grid.gridMin.y(), grid.cellSize.y(), grid.resolutionY);
    const size_t zMax = toIndex(segmentMax.z(), grid.gridMin.z(), grid.cellSize.z(), grid.resolutionZ);

    for (size_t x = xMin; x <= xMax; ++x)
    {
        for (size_t y = yMin; y <= yMax; ++y)
        {
            for (size_t z = zMin; z <= zMax; ++z)
            {
                for (const size_t triangleIndex : grid.cells[grid.cellIndex(x, y, z)])
                {
                    const auto& triangle = triangles_[triangleIndex];
                    if (!boundsOverlap(segmentMin, segmentMax, triangle.boundsMin, triangle.boundsMax))
                        continue;
                    if (RobustPredicates3D::segmentCrossesTriangle(a, b, triangle.vertices[0], triangle.vertices[1],
                                                                    triangle.vertices[2]))
                        return true;
                }
            }
        }
    }
    return false;
}

} // namespace Meshing
