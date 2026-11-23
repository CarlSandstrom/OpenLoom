#include <gtest/gtest.h>

#include "Common/Types.h"
#include "Meshing/Core/2D/General/NodeGrid2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/MeshMutator2D.h"
#include "Meshing/Data/Base/TriangleElement.h"

#include <algorithm>
#include <array>
#include <memory>
#include <vector>

using namespace Meshing;

namespace
{

bool contains(const std::vector<NodeId>& nodeIds, NodeId nodeId)
{
    return std::find(nodeIds.begin(), nodeIds.end(), nodeId) != nodeIds.end();
}

} // namespace

// A unit square of triangles plus a node no triangle uses far away, as
// Delaunay2D leaves the bounding triangle's corners (OPE-218).
class NodeGrid2DTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        MeshMutator2D mutator(meshData_);
        for (int row = 0; row <= 10; ++row)
            for (int column = 0; column <= 10; ++column)
                grid_[row][column] = mutator.addNode(Point2D(0.1 * column, 0.1 * row));
        for (int row = 0; row < 10; ++row)
            for (int column = 0; column < 10; ++column)
            {
                mutator.addElement(std::make_unique<TriangleElement>(
                    std::array<NodeId, 3>{grid_[row][column], grid_[row][column + 1], grid_[row + 1][column + 1]}));
                mutator.addElement(std::make_unique<TriangleElement>(
                    std::array<NodeId, 3>{grid_[row][column], grid_[row + 1][column + 1], grid_[row + 1][column]}));
            }
        farNode_ = mutator.addNode(Point2D(1000.0, -1000.0));
    }

    MeshData2D meshData_;
    std::array<std::array<NodeId, 11>, 11> grid_{};
    NodeId farNode_{0};
};

TEST_F(NodeGrid2DTest, SmallBoxReturnsTheNodesInsideButFewOthers)
{
    NodeGrid2D nodeGrid(meshData_);
    std::vector<NodeId> nodeIds;

    nodeGrid.collectNodesNear(Point2D(0.42, 0.42), Point2D(0.58, 0.58), nodeIds);

    EXPECT_TRUE(contains(nodeIds, grid_[5][5]));
    // The far node must not have stretched the grid over the whole square.
    EXPECT_LT(nodeIds.size(), 40u);
}

TEST_F(NodeGrid2DTest, NodeOutsideTheTrianglesIsStillFound)
{
    NodeGrid2D nodeGrid(meshData_);
    std::vector<NodeId> nodeIds;

    nodeGrid.collectNodesNear(Point2D(999.0, -1001.0), Point2D(1001.0, -999.0), nodeIds);

    EXPECT_TRUE(contains(nodeIds, farNode_));
}
