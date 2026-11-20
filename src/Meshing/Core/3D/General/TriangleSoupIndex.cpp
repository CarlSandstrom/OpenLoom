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

// The tolerance relative to the soup's scale (its extent, or its distance from
// the origin when that is larger): far above the rounding a cell walk
// accumulates, far below any cell size.
constexpr double TOLERANCE_FRACTION = 1e-9;

bool boundsOverlap(const Point3D& aMin, const Point3D& aMax, const Point3D& bMin, const Point3D& bMax);
size_t gridResolutionFor(size_t triangleCount);
std::array<double, 3> barycentricWeightsOf(const Point3D& point, const TriangleSoupIndex::Triangle& vertices);
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

    resolution_ = gridResolutionFor(triangles_.size());
    auto resolution = static_cast<double>(resolution_);
    gridMin_ = gridMin;
    gridMax_ = gridMax;

    // Cells are at most resolution_ times longer along one axis than along
    // another. Without the floor, a planar face whose coordinates differ only
    // by rounding would get cells narrower than that rounding, and no
    // floating-point walk could tell them apart.
    Point3D range = gridMax - gridMin;
    double largestRange = range.maxCoeff();
    double minimumCellSize = largestRange > 0.0 ? largestRange / (resolution * resolution) : 1.0;
    cellSize_ = (range / resolution).cwiseMax(minimumCellSize);

    double scale = std::max({largestRange, gridMin.cwiseAbs().maxCoeff(), gridMax.cwiseAbs().maxCoeff()});
    tolerance_ = TOLERANCE_FRACTION * (scale > 0.0 ? scale : 1.0);

    // Each triangle is registered with its bounding box grown by the tolerance,
    // so a walk that strays from the segment by less than that -- at a cell
    // corner, along a cell boundary, at an end -- still meets every triangle
    // the segment touches.
    cells_.resize(resolution_ * resolution_ * resolution_);
    Point3D margin = Point3D::Constant(tolerance_);
    for (size_t triangleIndex = 0; triangleIndex < triangles_.size(); ++triangleIndex)
    {
        auto& triangle = triangles_[triangleIndex];
        auto minimumCell = cellContaining(triangle.boundsMin - margin);
        auto maximumCell = cellContaining(triangle.boundsMax + margin);

        for (size_t x = minimumCell.x; x <= maximumCell.x; ++x)
            for (size_t y = minimumCell.y; y <= maximumCell.y; ++y)
                for (size_t z = minimumCell.z; z <= maximumCell.z; ++z)
                    cells_[cellIndex({x, y, z})].push_back(triangleIndex);
    }
}

// A triangle registered in more than one visited cell is tested more than once;
// the duplicate test is harmless (it finds the same crossing point again, so
// the nearest one is unaffected).
std::optional<TriangleSoupIndex::Crossing> TriangleSoupIndex::findCrossingNearest(const Point3D& a,
                                                                                  const Point3D& b,
                                                                                  const Point3D& target) const
{
    Point3D boundsMin = a.cwiseMin(b);
    Point3D boundsMax = a.cwiseMax(b);
    Point3D direction = b - a;

    std::optional<Crossing> nearest;
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
                nearest = Crossing{crossing, triangleIndex, barycentricWeightsOf(crossing, vertices)};
                nearestSquaredDistance = squaredDistance;
            }
        }
    return nearest;
}

// Within the tolerance, the walk is the exact traversal of a segment lying
// within the tolerance of (a, b): build() registered every triangle that far
// out, so the cells visited cannot miss one the segment touches.
std::vector<size_t> TriangleSoupIndex::cellsAlongSegment(const Point3D& a, const Point3D& b) const
{
    if (cells_.empty())
        return {};

    // Clip the segment's parameter range to the soup's bounds grown by the
    // tolerance (slab method).
    Point3D clipMin = gridMin_ - Point3D::Constant(tolerance_);
    Point3D clipMax = gridMax_ + Point3D::Constant(tolerance_);
    Point3D direction = b - a;
    double entry = 0.0;
    double exit = 1.0;
    for (int axis = 0; axis < 3; ++axis)
    {
        if (direction[axis] == 0.0)
        {
            if (a[axis] < clipMin[axis] || a[axis] > clipMax[axis])
                return {};
            continue;
        }
        double near = (clipMin[axis] - a[axis]) / direction[axis];
        double far = (clipMax[axis] - a[axis]) / direction[axis];
        if (near > far)
            std::swap(near, far);
        entry = std::max(entry, near);
        exit = std::min(exit, far);
        if (entry > exit)
            return {};
    }

    // Walk the cells the clipped segment passes through (Amanatides-Woo).
    auto startCell = cellContaining(a + entry * direction);
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

    // Only the segment's parameter ends the walk. Leaving the grid along one
    // axis before the end is rounding (the segment was clipped to it), so the
    // walk stays in the last cell along that axis and carries on along the others.
    auto resolution = static_cast<long long>(resolution_);
    std::vector<size_t> visited;
    while (true)
    {
        visited.push_back(cellIndex({static_cast<size_t>(cell[0]), static_cast<size_t>(cell[1]), static_cast<size_t>(cell[2])}));
        auto axis = static_cast<int>(std::min_element(nextBoundary.begin(), nextBoundary.end()) - nextBoundary.begin());
        if (nextBoundary[axis] > exit)
            break;
        if (cell[axis] + step[axis] < 0 || cell[axis] + step[axis] >= resolution)
        {
            nextBoundary[axis] = std::numeric_limits<double>::infinity();
            continue;
        }
        cell[axis] += step[axis];
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

// Weights of point, taken to lie in the triangle's plane, against its three
// vertices. All weight on the first vertex for a degenerate triangle.
std::array<double, 3> barycentricWeightsOf(const Point3D& point, const TriangleSoupIndex::Triangle& vertices)
{
    Point3D normal = (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]);
    double squaredArea = normal.squaredNorm();
    if (squaredArea == 0.0)
        return {1.0, 0.0, 0.0};

    double weight1 = normal.dot((point - vertices[0]).cross(vertices[2] - vertices[0])) / squaredArea;
    double weight2 = normal.dot((vertices[1] - vertices[0]).cross(point - vertices[0])) / squaredArea;
    return {1.0 - weight1 - weight2, weight1, weight2};
}

// Number of cells along each axis for a soup of this size, chosen so each cell
// holds roughly 1-4 triangles on average. The soup typically lies on a 2D
// surface, not filling a 3D volume, so many cells stay empty -- that's fine; a
// query only visits the cells along its own segment, not all cells.
size_t gridResolutionFor(size_t triangleCount)
{
    constexpr size_t MINIMUM_GRID_RESOLUTION = 4;
    constexpr size_t MAXIMUM_GRID_RESOLUTION = 50;
    auto fromCount = static_cast<size_t>(std::cbrt(static_cast<double>(triangleCount)));
    return std::clamp(fromCount, MINIMUM_GRID_RESOLUTION, MAXIMUM_GRID_RESOLUTION);
}

} // namespace

} // namespace Meshing
