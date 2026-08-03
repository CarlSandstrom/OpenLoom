#include "Meshing/Core/3D/RCDT/SurfaceTessellation.h"

#include "Common/BoundingBox2D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Core/3D/General/RobustPredicates3D.h"

namespace Meshing
{

void SurfaceTessellation::build(const Geometry3D::ISurface3D& surface, size_t samplesPerDirection)
{
    triangles_.clear();
    if (samplesPerDirection < 2)
        return;

    const auto bounds = surface.getParameterBounds();
    const double uMin = bounds.getUMin();
    const double uMax = bounds.getUMax();
    const double vMin = bounds.getVMin();
    const double vMax = bounds.getVMax();

    const size_t gridSize = samplesPerDirection + 1;

    const auto index = [gridSize](size_t i, size_t j)
    {
        return i * gridSize + j;
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

    std::vector<Point3D> gridPoints(gridSize * gridSize);
    std::vector<bool> withinTrim(gridSize * gridSize);

    for (size_t i = 0; i < gridSize; ++i)
    {
        const double u = uMin + (uMax - uMin) * (static_cast<double>(i) + GRID_JITTER_U) /
                                    (static_cast<double>(gridSize - 1) + GRID_JITTER_U);
        for (size_t j = 0; j < gridSize; ++j)
        {
            const double v = vMin + (vMax - vMin) * (static_cast<double>(j) + GRID_JITTER_V) /
                                        (static_cast<double>(gridSize - 1) + GRID_JITTER_V);
            const Point3D point = surface.getPoint(u, v);
            gridPoints[index(i, j)] = point;
            withinTrim[index(i, j)] = surface.isPointWithinTrimmedBoundary(point);
        }
    }

    for (size_t i = 0; i + 1 < gridSize; ++i)
    {
        for (size_t j = 0; j + 1 < gridSize; ++j)
        {
            const size_t i00 = index(i, j);
            const size_t i10 = index(i + 1, j);
            const size_t i01 = index(i, j + 1);
            const size_t i11 = index(i + 1, j + 1);

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

            triangles_.push_back({gridPoints[i00], gridPoints[i10], gridPoints[i11]});
            triangles_.push_back({gridPoints[i00], gridPoints[i11], gridPoints[i01]});
        }
    }
}

bool SurfaceTessellation::crossesSurface(const Point3D& a, const Point3D& b) const
{
    for (const auto& triangle : triangles_)
    {
        if (RobustPredicates3D::segmentCrossesTriangle(a, b, triangle[0], triangle[1], triangle[2]))
            return true;
    }
    return false;
}

} // namespace Meshing
