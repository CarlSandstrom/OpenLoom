#include "Meshing/Core/2D/General/NodeGrid2D.h"

#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/Node2D.h"

#include <algorithm>
#include <cmath>

namespace Meshing
{

NodeGrid2D::NodeGrid2D(const MeshData2D& meshData)
{
    auto& nodes = meshData.getNodes();
    if (nodes.empty()) return;

    // Sized by the nodes the triangles use. A node no triangle uses -- the
    // bounding triangle's corners stay behind, far outside the model (OPE-218)
    // -- would otherwise stretch the grid until the whole model fits in one
    // cell. It is still binned: cellContaining() clamps it into an edge cell.
    Point2D minimum = nodes.begin()->second->getCoordinates();
    Point2D maximum = minimum;
    bool anyElement = false;
    for (auto& [elementId, element] : meshData.getElements())
    {
        for (auto nodeId : element->getNodeIds())
        {
            auto& coordinates = meshData.getNode(nodeId)->getCoordinates();
            if (!anyElement) minimum = maximum = coordinates;
            anyElement = true;
            minimum = minimum.cwiseMin(coordinates);
            maximum = maximum.cwiseMax(coordinates);
        }
    }

    resolution_ = std::max<size_t>(1, static_cast<size_t>(std::sqrt(static_cast<double>(nodes.size()))));
    gridMinimum_ = minimum;
    Point2D range = maximum - minimum;
    for (int axis = 0; axis < 2; ++axis)
        cellSize_[axis] = range[axis] > 0.0 ? range[axis] / static_cast<double>(resolution_) : 1.0;

    cells_.resize(resolution_ * resolution_);
    for (auto& [nodeId, node] : nodes)
    {
        auto cell = cellContaining(node->getCoordinates());
        cells_[cell.x * resolution_ + cell.y].push_back(nodeId);
    }
}

// cellContaining() is monotone in each coordinate, clamping included, so every
// node inside the box lies in a cell between the box corners' cells.
void NodeGrid2D::collectNodesNear(const Point2D& minimum, const Point2D& maximum, std::vector<NodeId>& nodeIds) const
{
    if (cells_.empty()) return;

    auto minimumCell = cellContaining(minimum);
    auto maximumCell = cellContaining(maximum);
    for (size_t x = minimumCell.x; x <= maximumCell.x; ++x)
        for (size_t y = minimumCell.y; y <= maximumCell.y; ++y)
        {
            auto& cell = cells_[x * resolution_ + y];
            nodeIds.insert(nodeIds.end(), cell.begin(), cell.end());
        }
}

NodeGrid2D::CellCoordinates NodeGrid2D::cellContaining(const Point2D& point) const
{
    auto axisIndex = [this, &point](int axis) -> size_t
    {
        auto index = static_cast<long long>(std::floor((point[axis] - gridMinimum_[axis]) / cellSize_[axis]));
        return static_cast<size_t>(std::clamp(index, 0LL, static_cast<long long>(resolution_) - 1LL));
    };
    return {axisIndex(0), axisIndex(1)};
}

} // namespace Meshing
