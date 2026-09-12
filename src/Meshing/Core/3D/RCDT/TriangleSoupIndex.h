#pragma once

#include "Common/Types.h"

#include <array>
#include <cstddef>
#include <vector>

namespace Meshing
{

/**
 * @brief A fixed set of triangles with a uniform spatial grid over them,
 * answering "does this segment cross any of them?" exactly.
 *
 * Nothing here knows what the triangles approximate: they are an unstructured
 * soup of independent triangles (no shared-vertex or adjacency information),
 * bounded by their own extent, and the only query is the exact segment
 * crossing test. SurfaceTessellation builds one per CAD surface, but the index
 * itself is general.
 *
 * Without the grid, each query would scan every triangle. Each triangle is
 * instead registered in every cell its bounding box overlaps, and a query
 * visits only the cells the segment's own bounding box overlaps. For the short
 * segments that dominate RCDT refinement (dual Voronoi edges) that is a small
 * fraction of the cells, so the cost drops from the whole soup to the handful
 * of triangles near the segment.
 *
 * Built in one call rather than triangle by triangle, so there is no
 * half-built state in which a query could silently miss triangles.
 */
class TriangleSoupIndex
{
public:
    using Triangle = std::array<Point3D, 3>;

    /// Replaces the contents with these triangles and indexes them.
    void build(const std::vector<Triangle>& triangles);

    /// Whether segment (a, b) crosses any triangle in the soup — exact,
    /// checking RobustPredicates3D::segmentCrossesTriangle against every
    /// triangle whose axis-aligned bounding box the segment's own bounding box
    /// overlaps (a triangle outside that can't possibly be crossed, so the
    /// exact -- much more expensive -- predicate only runs on candidates that
    /// survive this cheap prefilter).
    bool isCrossedBySegment(const Point3D& a, const Point3D& b) const;

private:
    struct BoundedTriangle
    {
        Triangle vertices;
        Point3D boundsMin;
        Point3D boundsMax;
    };

    /// A query segment plus its own bounding box, computed once per query
    /// rather than once per candidate triangle.
    struct SegmentQuery
    {
        Point3D start;
        Point3D end;
        Point3D boundsMin;
        Point3D boundsMax;
    };

    struct CellCoordinates
    {
        size_t x;
        size_t y;
        size_t z;
    };

    /// The cell containing point, clamped to the grid for a point outside it.
    CellCoordinates cellContaining(const Point3D& point) const;
    size_t cellIndex(const CellCoordinates& cell) const;
    bool anyTriangleInCellCrosses(size_t cellIndex, const SegmentQuery& segment) const;

    std::vector<BoundedTriangle> triangles_;

    // The grid: cubic, resolution_ cells along each axis, cell (x, y, z)
    // spanning gridMin_ + cellSize_ * (x, y, z) and one cellSize_ further.
    // Each cell holds the indices into triangles_ of the triangles whose
    // bounding box overlaps it, so a triangle spanning a cell boundary appears
    // in more than one cell. Empty until build() runs, and stays empty for an
    // empty soup.
    Point3D gridMin_ = Point3D::Zero();
    Point3D cellSize_ = Point3D::Ones();
    size_t resolution_ = 0;
    std::vector<std::vector<size_t>> cells_; // indexed by cellIndex()
};

} // namespace Meshing
