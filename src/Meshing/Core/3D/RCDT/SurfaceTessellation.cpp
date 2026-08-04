#include "Meshing/Core/3D/RCDT/SurfaceTessellation.h"

#include "Common/BoundingBox2D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Core/3D/General/RobustPredicates3D.h"

#include <algorithm>
#include <cmath>
#include <optional>

namespace Meshing
{

namespace
{

// How much smaller than requiredEdgeLength characteristicCellSize_ must be
// before ensureResolution() considers the tessellation fine enough.
constexpr double CELL_SIZE_SAFETY_MARGIN = 0.5;

// Upper bound on samplesPerDirection a rebuild can request, bounding worst-
// case tessellation memory/cost the same way minimumEdgeLength_ bounds
// refinement elsewhere -- not perfect, but bounded.
constexpr size_t MAXIMUM_SAMPLES_PER_DIRECTION = 400;

// A query segment much longer than the surface's own physical size isn't a
// real local dual edge -- it's RestrictedTriangulation's bounding-supertet-
// node fallback (see computeDualEdgeEndpoint()), whose far endpoint can be
// hundreds of units away. Letting a segment like that drive a resolution
// upgrade doesn't reflect real local mesh density and wastes a rebuild on
// a case ensureResolution can't actually help. A generous multiple of the
// surface's diameter (not 1x) so a segment that's merely long because it
// legitimately spans a large curved surface isn't mistaken for this.
constexpr double MAXIMUM_SEGMENT_LENGTH_RELATIVE_TO_DIAMETER = 5.0;

// Rough characteristic size of the surface's parameter-bounds footprint, used
// only to turn the periodicity probe below into a relative tolerance. Reuses
// the same 4-corner-diameter idea as SurfaceProjector::computeSurfaceDiameter.
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

// Whether two axis-aligned boxes [aMin, aMax] and [bMin, bMax] overlap in all
// 3 axes. A necessary (not sufficient) condition for the shapes they bound to
// actually intersect -- see crossesSurface().
bool boundsOverlap(const Point3D& aMin, const Point3D& aMax, const Point3D& bMin, const Point3D& bMax)
{
    return aMin.x() <= bMax.x() && bMin.x() <= aMax.x() && aMin.y() <= bMax.y() && bMin.y() <= aMax.y() &&
           aMin.z() <= bMax.z() && bMin.z() <= aMax.z();
}

// Whether surface's normal is (numerically) identical across a jittered
// grid of probe points spanning its parameter bounds. For a genuinely flat
// surface a tessellation represents the surface exactly no matter how
// coarse it is -- ensureResolution() has no work to do there, ever. This
// matters in practice, not just in principle: a surface's untrimmed
// parameter-bounds footprint can be far larger than its actual trimmed
// patch (common for CAD -- e.g. a fillet's underlying cylinder), which
// would otherwise make characteristicCellSize_ look coarse relative to a
// small mesh element and trigger a large, entirely unnecessary rebuild on
// a surface where resolution was never the issue. Any real curvature
// produces normal variation many orders of magnitude above the tolerance
// here, even over a tiny patch -- as long as a probe lands where the slope
// is actually nonzero. A localized, symmetric feature (e.g. a bump) has
// zero slope exactly at its own center by construction, so corners/center
// alone can be fooled into reading it as flat; jittering the grid (same
// idea as the tessellation grid itself, see build()) makes landing exactly
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

void SurfaceTessellation::build(const Geometry3D::ISurface3D& surface, size_t samplesPerDirection)
{
    triangles_.clear();
    surface_ = &surface;
    samplesPerDirection_ = samplesPerDirection;
    characteristicCellSize_ = 0.0;
    if (samplesPerDirection < 2)
        return;

    const auto bounds = surface.getParameterBounds();
    const double uMin = bounds.getUMin();
    const double uMax = bounds.getUMax();
    const double vMin = bounds.getVMin();
    const double vMax = bounds.getVMax();

    isFlat_ = isEffectivelyFlat(surface, uMin, uMax, vMin, vMax);

    // ISurface3D has no explicit periodicity query, so detect it numerically:
    // a periodic direction's two parameter extremes map to the same physical
    // point (probed at the other direction's midpoint, to sidestep any
    // corner/pole degeneracy). Works uniformly for any backend, not just OCC.
    // This matters because a plain non-wrapping grid leaves a real gap right
    // along the seam of a periodic surface (e.g. a torus's major-circle seam,
    // OPE-171): nothing samples exactly the boundary column *and* its twin at
    // the other end of the period, so no triangle ever covers the strip
    // between the last sampled column and the first.
    const double diameter = estimateDiameter(surface, uMin, uMax, vMin, vMax);
    surfaceDiameter_ = diameter;
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
            withinTrim[index(i, j)] = surface.isPointWithinTrimmedBoundary(point);
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

    for (const auto& triangle : triangles_)
    {
        const auto& v = triangle.vertices;
        characteristicCellSize_ = std::max({characteristicCellSize_, (v[1] - v[0]).norm(), (v[2] - v[1]).norm(),
                                            (v[0] - v[2]).norm()});
    }
}

void SurfaceTessellation::ensureResolution(const Point3D& a, const Point3D& b, double requiredEdgeLength)
{
    if (!surface_ || isFlat_ || characteristicCellSize_ <= 0.0 || requiredEdgeLength <= 0.0)
        return;
    if (surfaceDiameter_ > 0.0 && (b - a).norm() > MAXIMUM_SEGMENT_LENGTH_RELATIVE_TO_DIAMETER * surfaceDiameter_)
        return;

    const double targetCellSize = requiredEdgeLength * CELL_SIZE_SAFETY_MARGIN;
    if (characteristicCellSize_ <= targetCellSize)
        return;

    const double scaleFactor = characteristicCellSize_ / targetCellSize;
    const size_t newSamples = static_cast<size_t>(
        std::ceil(static_cast<double>(samplesPerDirection_) * scaleFactor));

    build(*surface_, std::min(newSamples, MAXIMUM_SAMPLES_PER_DIRECTION));
}

void SurfaceTessellation::addTriangle(const Point3D& p0, const Point3D& p1, const Point3D& p2)
{
    const Point3D boundsMin = p0.cwiseMin(p1).cwiseMin(p2);
    const Point3D boundsMax = p0.cwiseMax(p1).cwiseMax(p2);
    triangles_.push_back({{p0, p1, p2}, boundsMin, boundsMax});
}

bool SurfaceTessellation::crossesSurface(const Point3D& a, const Point3D& b) const
{
    const Point3D segmentMin = a.cwiseMin(b);
    const Point3D segmentMax = a.cwiseMax(b);

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

} // namespace Meshing
