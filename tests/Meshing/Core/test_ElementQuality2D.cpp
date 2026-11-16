#include <gtest/gtest.h>

#include "Common/Types.h"
#include "Meshing/Core/2D/General/ElementQuality2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/MeshMutator2D.h"
#include "Meshing/Data/Base/TriangleElement.h"

#include <cmath>

using namespace Meshing;

namespace
{
constexpr double TOLERANCE = 1e-9;
}

class ElementQuality2DTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        mutator_ = std::make_unique<MeshMutator2D>(meshData_);
    }

    NodeId addNode(double x, double y)
    {
        return mutator_->addNode(Point2D(x, y));
    }

    ElementId addTriangle(NodeId n0, NodeId n1, NodeId n2)
    {
        return mutator_->addElement(
            std::make_unique<TriangleElement>(std::array<NodeId, 3>{n0, n1, n2}));
    }

    MeshData2D meshData_;
    std::unique_ptr<MeshMutator2D> mutator_;
};

TEST_F(ElementQuality2DTest, ComputeShortestEdgeLengthForEquilateralTriangle)
{
    NodeId n0 = addNode(0.0, 0.0);
    NodeId n1 = addNode(1.0, 0.0);
    NodeId n2 = addNode(0.5, std::sqrt(3.0) / 2.0);
    addTriangle(n0, n1, n2);

    ElementQuality2D quality(meshData_);
    const auto* element = dynamic_cast<const TriangleElement*>(meshData_.getElement(0));

    double shortest = quality.computeShortestEdgeLength(*element);

    EXPECT_NEAR(shortest, 1.0, TOLERANCE);
}

TEST_F(ElementQuality2DTest, ComputeCircumradiusToShortestEdgeRatioForEquilateral)
{
    NodeId n0 = addNode(0.0, 0.0);
    NodeId n1 = addNode(1.0, 0.0);
    NodeId n2 = addNode(0.5, std::sqrt(3.0) / 2.0);
    addTriangle(n0, n1, n2);

    ElementQuality2D quality(meshData_);
    const auto* element = dynamic_cast<const TriangleElement*>(meshData_.getElement(0));

    auto ratio = quality.computeCircumradiusToShortestEdgeRatio(*element);

    ASSERT_TRUE(ratio.has_value());
    EXPECT_NEAR(ratio.value(), 1.0 / std::sqrt(3.0), TOLERANCE);
}

TEST_F(ElementQuality2DTest, ComputeCircumradiusToShortestEdgeRatioForSkinnyTriangle)
{
    NodeId n0 = addNode(0.0, 0.0);
    NodeId n1 = addNode(10.0, 0.0);
    NodeId n2 = addNode(5.0, 0.1);
    addTriangle(n0, n1, n2);

    ElementQuality2D quality(meshData_);
    const auto* element = dynamic_cast<const TriangleElement*>(meshData_.getElement(0));

    auto ratio = quality.computeCircumradiusToShortestEdgeRatio(*element);

    ASSERT_TRUE(ratio.has_value());
    // Skinny triangles have high circumradius-to-shortest-edge ratios
    EXPECT_GT(ratio.value(), 1.0);
}

TEST_F(ElementQuality2DTest, GetTrianglesSortedByQualityReturnsWorstFirst)
{
    NodeId n0 = addNode(0.0, 0.0);
    NodeId n1 = addNode(1.0, 0.0);
    NodeId n2 = addNode(0.5, std::sqrt(3.0) / 2.0);
    ElementId goodTriangleId = addTriangle(n0, n1, n2);

    NodeId n3 = addNode(10.0, 0.0);
    NodeId n4 = addNode(20.0, 0.0);
    NodeId n5 = addNode(15.0, 0.1);
    ElementId skinnyTriangleId = addTriangle(n3, n4, n5);

    ElementQuality2D quality(meshData_);

    auto sorted = quality.getTrianglesSortedByQuality();

    ASSERT_EQ(sorted.size(), 2u);
    EXPECT_EQ(sorted[0], skinnyTriangleId);
    EXPECT_EQ(sorted[1], goodTriangleId);
}

TEST_F(ElementQuality2DTest, GetTrianglesSortedByQualityHandlesEmptyMesh)
{
    ElementQuality2D quality(meshData_);

    auto sorted = quality.getTrianglesSortedByQuality();

    EXPECT_TRUE(sorted.empty());
}
