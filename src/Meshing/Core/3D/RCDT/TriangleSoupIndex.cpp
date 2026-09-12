#include "Meshing/Core/3D/RCDT/TriangleSoupIndex.h"

#include "Meshing/Core/3D/General/RobustPredicates3D.h"

#include <algorithm>
#include <cmath>

namespace Meshing
{

namespace
{

// Whether two axis-aligned boxes [aMin, aMax] and [bMin, bMax] overlap in all
// 3 axes. A necessary (not sufficient) condition for the shapes they bound to
// actually intersect -- see isCrossedBySegment().
bool boundsOverlap(const Point3D& aMin, const Point3D& aMax, const Point3D& bMin, const Point3D& bMax)
{
    return aMin.x() <= bMax.x() && bMin.x() <= aMax.x() && aMin.y() <= bMax.y() && bMin.y() <= aMax.y() &&
           aMin.z() <= bMax.z() && bMin.z() <= aMax.z();
}

// Number of cells along each axis for a soup of this size, chosen so each cell
// holds roughly 1-4 triangles on average. The soup typically lies on a 2D
// surface, not filling a 3D volume, so many cells stay empty -- that's fine; a
// query only visits cells overlapping its own bounding box, not all cells.
size_t gridResolutionFor(size_t triangleCount)
{
    constexpr size_t MINIMUM_GRID_RESOLUTION = 4;
    constexpr size_t MAXIMUM_GRID_RESOLUTION = 50;
    const size_t fromCount = static_cast<size_t>(std::cbrt(static_cast<double>(triangleCount)));
    return std::clamp(fromCount, MINIMUM_GRID_RESOLUTION, MAXIMUM_GRID_RESOLUTION);
}

} // namespace

void TriangleSoupIndex::build(const std::vector<Triangle>& triangles)
{
    triangles_.clear();
    cells_.clear();
    resolution_ = 0;
    if (triangles.empty())
        return;

    triangles_.reserve(triangles.size());
    for (const Triangle& vertices : triangles)
    {
        const Point3D boundsMin = vertices[0].cwiseMin(vertices[1]).cwiseMin(vertices[2]);
        const Point3D boundsMax = vertices[0].cwiseMax(vertices[1]).cwiseMax(vertices[2]);
        triangles_.push_back({vertices, boundsMin, boundsMax});
    }

    Point3D gridMin = triangles_[0].boundsMin;
    Point3D gridMax = triangles_[0].boundsMax;
    for (const BoundedTriangle& triangle : triangles_)
    {
        gridMin = gridMin.cwiseMin(triangle.boundsMin);
        gridMax = gridMax.cwiseMax(triangle.boundsMax);
    }

    // Expand slightly so that triangles exactly on the boundary fall inside a
    // cell rather than rounding to an out-of-range index.
    constexpr double GRID_EPSILON = 1e-10;
    gridMin -= Point3D::Constant(GRID_EPSILON);
    gridMax += Point3D::Constant(GRID_EPSILON);

    resolution_ = gridResolutionFor(triangles_.size());
    gridMin_ = gridMin;

    const Point3D range = gridMax - gridMin;
    // Floor each component at a small positive value to avoid division by zero
    // for a soup that is flat in one coordinate direction.
    constexpr double MINIMUM_RANGE = 1e-12;
    const double cellCount = static_cast<double>(resolution_);
    cellSize_ = Point3D(std::max(range.x() / cellCount, MINIMUM_RANGE), std::max(range.y() / cellCount, MINIMUM_RANGE),
                        std::max(range.z() / cellCount, MINIMUM_RANGE));

    cells_.resize(resolution_ * resolution_ * resolution_);

    for (size_t triangleIndex = 0; triangleIndex < triangles_.size(); ++triangleIndex)
    {
        const BoundedTriangle& triangle = triangles_[triangleIndex];
        const CellCoordinates minimumCell = cellContaining(triangle.boundsMin);
        const CellCoordinates maximumCell = cellContaining(triangle.boundsMax);

        for (size_t x = minimumCell.x; x <= maximumCell.x; ++x)
            for (size_t y = minimumCell.y; y <= maximumCell.y; ++y)
                for (size_t z = minimumCell.z; z <= maximumCell.z; ++z)
                    cells_[cellIndex({x, y, z})].push_back(triangleIndex);
    }
}

bool TriangleSoupIndex::isCrossedBySegment(const Point3D& a, const Point3D& b) const
{
    if (cells_.empty())
        return false;

    const SegmentQuery segment = {a, b, a.cwiseMin(b), a.cwiseMax(b)};

    const CellCoordinates minimumCell = cellContaining(segment.boundsMin);
    const CellCoordinates maximumCell = cellContaining(segment.boundsMax);

    for (size_t x = minimumCell.x; x <= maximumCell.x; ++x)
        for (size_t y = minimumCell.y; y <= maximumCell.y; ++y)
            for (size_t z = minimumCell.z; z <= maximumCell.z; ++z)
                if (anyTriangleInCellCrosses(cellIndex({x, y, z}), segment))
                    return true;

    return false;
}

// A triangle registered in more than one visited cell is tested more than once;
// the duplicate test is harmless (at worst a redundant true that terminates the
// search anyway, or a redundant false that wastes a little work).
bool TriangleSoupIndex::anyTriangleInCellCrosses(size_t cellIndex, const SegmentQuery& segment) const
{
    for (const size_t triangleIndex : cells_[cellIndex])
    {
        const BoundedTriangle& triangle = triangles_[triangleIndex];
        if (!boundsOverlap(segment.boundsMin, segment.boundsMax, triangle.boundsMin, triangle.boundsMax))
            continue;
        if (RobustPredicates3D::segmentCrossesTriangle(segment.start, segment.end, triangle.vertices[0],
                                                       triangle.vertices[1], triangle.vertices[2]))
            return true;
    }
    return false;
}

TriangleSoupIndex::CellCoordinates TriangleSoupIndex::cellContaining(const Point3D& point) const
{
    const auto axisIndex = [this](double coordinate, double gridMinCoordinate, double cellSizeCoordinate) -> size_t
    {
        const double normalized = (coordinate - gridMinCoordinate) / cellSizeCoordinate;
        const long long index = static_cast<long long>(std::floor(normalized));
        return static_cast<size_t>(std::clamp(index, 0LL, static_cast<long long>(resolution_) - 1LL));
    };

    return {axisIndex(point.x(), gridMin_.x(), cellSize_.x()), axisIndex(point.y(), gridMin_.y(), cellSize_.y()),
            axisIndex(point.z(), gridMin_.z(), cellSize_.z())};
}

size_t TriangleSoupIndex::cellIndex(const CellCoordinates& cell) const
{
    return cell.x * resolution_ * resolution_ + cell.y * resolution_ + cell.z;
}

} // namespace Meshing
