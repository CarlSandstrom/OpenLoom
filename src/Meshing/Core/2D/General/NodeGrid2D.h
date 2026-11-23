#pragma once

#include "Common/Types.h"

#include <cstddef>
#include <vector>

namespace Meshing
{

class MeshData2D;

/**
 * @brief The mesh's nodes binned into a uniform grid, built in one pass.
 *
 * Answers "which nodes may lie in this box" without scanning every node: about
 * sqrt(N) cells along each axis, so a box the size of a few elements touches a
 * handful of cells. It is a snapshot -- nodes added afterwards are not in it.
 */
class NodeGrid2D
{
public:
    explicit NodeGrid2D(const MeshData2D& meshData);

    /// Appends every node lying in the closed box [minimum, maximum] to nodeIds,
    /// together with others from the same cells; the caller tests each one.
    void collectNodesNear(const Point2D& minimum, const Point2D& maximum, std::vector<NodeId>& nodeIds) const;

private:
    struct CellCoordinates
    {
        size_t x;
        size_t y;
    };

    CellCoordinates cellContaining(const Point2D& point) const;

    Point2D gridMinimum_ = Point2D::Zero();
    Point2D cellSize_ = Point2D::Ones();
    size_t resolution_ = 0;
    std::vector<std::vector<NodeId>> cells_; // row-major, x * resolution_ + y
};

} // namespace Meshing
