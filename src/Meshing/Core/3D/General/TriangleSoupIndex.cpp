#include "Meshing/Core/3D/General/TriangleSoupIndex.h"

#include "Meshing/Core/3D/General/RobustPredicates3D.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace Meshing
{

namespace
{
bool boundsOverlap(const Point3D& aMin, const Point3D& aMax, const Point3D& bMin, const Point3D& bMax);
size_t gridResolutionFor(size_t triangleCount);
} // namespace

void TriangleSoupIndex::build(const std::vector<Triangle>& triangles)
{
    triangles_.clear();
    cells_.clear();
    resolution_ = 0;
    if (triangles.empty())
        return;

    triangles_.reserve(triangles.size());
    Point3D gridMin = triangles[0][0];
    Point3D gridMax = triangles[0][0];
    for (auto& vertices : triangles)
    {
        Point3D boundsMin = vertices[0].cwiseMin(vertices[1]).cwiseMin(vertices[2]);
        Point3D boundsMax = vertices[0].cwiseMax(vertices[1]).cwiseMax(vertices[2]);
        triangles_.push_back({vertices, boundsMin, boundsMax});
        gridMin = gridMin.cwiseMin(boundsMin);
        gridMax = gridMax.cwiseMax(boundsMax);
    }

    // Expand slightly so that triangles exactly on the boundary fall inside a
    // cell rather than rounding to an out-of-range index.
    constexpr double GRID_EPSILON = 1e-10;
    gridMin -= Point3D::Constant(GRID_EPSILON);
    gridMax += Point3D::Constant(GRID_EPSILON);

    resolution_ = gridResolutionFor(triangles_.size());
    gridMin_ = gridMin;

    Point3D range = gridMax - gridMin;
    // Floor each component at a small positive value to avoid division by zero
    // for a soup that is flat in one coordinate direction.
    constexpr double MINIMUM_RANGE = 1e-12;
    cellSize_ = (range / static_cast<double>(resolution_)).cwiseMax(MINIMUM_RANGE);

    cells_.resize(resolution_ * resolution_ * resolution_);

    for (size_t triangleIndex = 0; triangleIndex < triangles_.size(); ++triangleIndex)
    {
        auto& triangle = triangles_[triangleIndex];
        auto minimumCell = cellContaining(triangle.boundsMin);
        auto maximumCell = cellContaining(triangle.boundsMax);

        for (size_t x = minimumCell.x; x <= maximumCell.x; ++x)
            for (size_t y = minimumCell.y; y <= maximumCell.y; ++y)
                for (size_t z = minimumCell.z; z <= maximumCell.z; ++z)
                    cells_[cellIndex({x, y, z})].push_back(triangleIndex);
    }
}

// A triangle registered in more than one visited cell is tested more than once;
// the duplicate test is harmless (it finds the same crossing point again, so
// the nearest one is unaffected).
std::optional<Point3D> TriangleSoupIndex::findCrossingNearest(const Point3D& a,
                                                              const Point3D& b,
                                                              const Point3D& target) const
{
    Point3D boundsMin = a.cwiseMin(b);
    Point3D boundsMax = a.cwiseMax(b);
    Point3D direction = b - a;

    std::optional<Point3D> nearest;
    double nearestSquaredDistance = 0.0;
    for (auto cell : cellsAlongSegment(a, b))
        for (auto triangleIndex : cells_[cell])
        {
            auto& triangle = triangles_[triangleIndex];
            if (!boundsOverlap(boundsMin, boundsMax, triangle.boundsMin, triangle.boundsMax))
                continue;
            auto& vertices = triangle.vertices;
            if (!RobustPredicates3D::segmentCrossesTriangle(a, b, vertices[0], vertices[1], vertices[2]))
                continue;
            Point3D normal = (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]);
            double denominator = normal.dot(direction);
            double parameter = denominator == 0.0 ? 0.0 : normal.dot(vertices[0] - a) / denominator;
            Point3D crossing = a + std::clamp(parameter, 0.0, 1.0) * direction;
            double squaredDistance = (crossing - target).squaredNorm();
            if (!nearest || squaredDistance < nearestSquaredDistance)
            {
                nearest = crossing;
                nearestSquaredDistance = squaredDistance;
            }
        }
    return nearest;
}

std::vector<size_t> TriangleSoupIndex::cellsAlongSegment(const Point3D& a, const Point3D& b) const
{
    if (cells_.empty())
        return {};

    // Clip the segment's parameter range to the grid box (slab method).
    Point3D direction = b - a;
    Point3D gridMax = gridMin_ + cellSize_ * static_cast<double>(resolution_);
    double entry = 0.0;
    double exit = 1.0;
    for (int axis = 0; axis < 3; ++axis)
    {
        if (direction[axis] == 0.0)
        {
            if (a[axis] < gridMin_[axis] || a[axis] > gridMax[axis])
                return {};
            continue;
        }
        double near = (gridMin_[axis] - a[axis]) / direction[axis];
        double far = (gridMax[axis] - a[axis]) / direction[axis];
        if (near > far)
            std::swap(near, far);
        entry = std::max(entry, near);
        exit = std::min(exit, far);
        if (entry > exit)
            return {};
    }

    // Walk the cells the clipped segment passes through (Amanatides-Woo).
    Point3D start = a + entry * direction;
    auto startCell = cellContaining(start);
    std::array<long long, 3> cell = {static_cast<long long>(startCell.x), static_cast<long long>(startCell.y),
                                     static_cast<long long>(startCell.z)};
    std::array<long long, 3> step{};
    std::array<double, 3> nextBoundary{};
    std::array<double, 3> boundarySpacing{};
    for (int axis = 0; axis < 3; ++axis)
    {
        if (direction[axis] == 0.0)
        {
            nextBoundary[axis] = std::numeric_limits<double>::infinity();
            boundarySpacing[axis] = std::numeric_limits<double>::infinity();
            continue;
        }
        step[axis] = direction[axis] > 0.0 ? 1 : -1;
        double boundary = gridMin_[axis] + cellSize_[axis] * static_cast<double>(cell[axis] + (step[axis] > 0 ? 1 : 0));
        nextBoundary[axis] = (boundary - a[axis]) / direction[axis];
        boundarySpacing[axis] = cellSize_[axis] / std::abs(direction[axis]);
    }

    auto resolution = static_cast<long long>(resolution_);
    std::vector<size_t> visited;
    while (true)
    {
        visited.push_back(cellIndex({static_cast<size_t>(cell[0]), static_cast<size_t>(cell[1]), static_cast<size_t>(cell[2])}));
        auto axis = static_cast<int>(std::min_element(nextBoundary.begin(), nextBoundary.end()) - nextBoundary.begin());
        if (nextBoundary[axis] > exit)
            break;
        cell[axis] += step[axis];
        if (cell[axis] < 0 || cell[axis] >= resolution)
            break;
        nextBoundary[axis] += boundarySpacing[axis];
    }
    return visited;
}

TriangleSoupIndex::CellCoordinates TriangleSoupIndex::cellContaining(const Point3D& point) const
{
    auto axisIndex = [this, &point](int axis) -> size_t
    {
        auto index = static_cast<long long>(std::floor((point[axis] - gridMin_[axis]) / cellSize_[axis]));
        return static_cast<size_t>(std::clamp(index, 0LL, static_cast<long long>(resolution_) - 1LL));
    };

    return {axisIndex(0), axisIndex(1), axisIndex(2)};
}

size_t TriangleSoupIndex::cellIndex(const CellCoordinates& cell) const
{
    return cell.x * resolution_ * resolution_ + cell.y * resolution_ + cell.z;
}

namespace
{

// Whether two axis-aligned boxes [aMin, aMax] and [bMin, bMax] overlap in all
// 3 axes. A necessary (not sufficient) condition for the shapes they bound to
// actually intersect -- see findCrossingNearest().
bool boundsOverlap(const Point3D& aMin, const Point3D& aMax, const Point3D& bMin, const Point3D& bMax)
{
    return (aMin.array() <= bMax.array()).all() && (bMin.array() <= aMax.array()).all();
}

// Number of cells along each axis for a soup of this size, chosen so each cell
// holds roughly 1-4 triangles on average. The soup typically lies on a 2D
// surface, not filling a 3D volume, so many cells stay empty -- that's fine; a
// query only visits cells overlapping its own bounding box, not all cells.
size_t gridResolutionFor(size_t triangleCount)
{
    constexpr size_t MINIMUM_GRID_RESOLUTION = 4;
    constexpr size_t MAXIMUM_GRID_RESOLUTION = 50;
    auto fromCount = static_cast<size_t>(std::cbrt(static_cast<double>(triangleCount)));
    return std::clamp(fromCount, MINIMUM_GRID_RESOLUTION, MAXIMUM_GRID_RESOLUTION);
}

} // namespace

} // namespace Meshing
