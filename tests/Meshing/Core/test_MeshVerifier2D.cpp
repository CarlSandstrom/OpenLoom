#include <gtest/gtest.h>

#include "Common/Types.h"
#include "Meshing/Core/2D/General/MeshVerifier2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/MeshMutator2D.h"
#include "Meshing/Data/Base/TriangleElement.h"

#include <array>
#include <memory>

using namespace Meshing;

class MeshVerifier2DTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        mutator_ = std::make_unique<MeshMutator2D>(meshData_);
    }

    // Adds a counter-clockwise triangle with its own three nodes.
    void addTriangle(const Point2D& point0, const Point2D& point1, const Point2D& point2)
    {
        addTriangle(mutator_->addNode(point0), mutator_->addNode(point1), mutator_->addNode(point2));
    }

    void addTriangle(NodeId node0, NodeId node1, NodeId node2)
    {
        mutator_->addElement(std::make_unique<TriangleElement>(std::array<NodeId, 3>{node0, node1, node2}));
    }

    MeshData2D meshData_;
    std::unique_ptr<MeshMutator2D> mutator_;
};

TEST_F(MeshVerifier2DTest, TrianglesSharingAnEdgeAreValid)
{
    auto node0 = mutator_->addNode(Point2D(0.0, 0.0));
    auto node1 = mutator_->addNode(Point2D(1.0, 0.0));
    auto node2 = mutator_->addNode(Point2D(1.0, 1.0));
    auto node3 = mutator_->addNode(Point2D(0.0, 1.0));
    addTriangle(node0, node1, node2);
    addTriangle(node0, node2, node3);

    auto result = MeshVerifier2D::verify(meshData_);

    EXPECT_TRUE(result.isValid);
    EXPECT_TRUE(result.errors.empty());
}

TEST_F(MeshVerifier2DTest, TrianglesTouchingAtACornerAreValid)
{
    addTriangle(Point2D(0.0, 0.0), Point2D(1.0, 0.0), Point2D(1.0, 1.0));
    addTriangle(Point2D(1.0, 1.0), Point2D(2.0, 1.0), Point2D(2.0, 2.0));

    auto result = MeshVerifier2D::verify(meshData_);

    EXPECT_TRUE(result.isValid);
}

TEST_F(MeshVerifier2DTest, OverlappingTrianglesAreReported)
{
    addTriangle(Point2D(0.0, 0.0), Point2D(2.0, 0.0), Point2D(0.0, 2.0));
    addTriangle(Point2D(0.5, 0.5), Point2D(3.0, 0.5), Point2D(0.5, 3.0));

    auto result = MeshVerifier2D::verify(meshData_);

    EXPECT_FALSE(result.isValid);
    EXPECT_EQ(result.errors.size(), 1u);
}

// A long triangle overlaps one whose x-extent starts well after it, with disjoint triangles in
// between in x order. The overlap must still be found: the sweep may only stop scanning once a
// triangle starts past the long one's end.
TEST_F(MeshVerifier2DTest, OverlapIsFoundPastNonOverlappingTrianglesInXOrder)
{
    addTriangle(Point2D(0.0, 0.0), Point2D(10.0, 0.0), Point2D(10.0, 1.0));
    addTriangle(Point2D(2.0, 5.0), Point2D(3.0, 5.0), Point2D(3.0, 6.0));
    addTriangle(Point2D(4.0, 5.0), Point2D(5.0, 5.0), Point2D(5.0, 6.0));
    addTriangle(Point2D(8.0, 0.5), Point2D(9.0, -1.0), Point2D(9.0, 2.0));

    auto result = MeshVerifier2D::verify(meshData_);

    EXPECT_FALSE(result.isValid);
    EXPECT_EQ(result.errors.size(), 1u);
}
