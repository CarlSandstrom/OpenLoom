#include <gtest/gtest.h>

#include "Common/Types.h"
#include "Meshing/Core/2D/Computer2D.h"
#include "Meshing/Core/2D/GeometryStructures2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/MeshMutator2D.h"
#include "Meshing/Data/2D/TriangleElement.h"

#include <cmath>

using namespace Meshing;

namespace
{
constexpr double TOLERANCE = 1e-9;
}

class Computer2DTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        mutator_ = std::make_unique<MeshMutator2D>(meshData_);
    }

    size_t addNode(double x, double y)
    {
        return mutator_->addNode(Point2D(x, y));
    }

    size_t addTriangle(size_t n0, size_t n1, size_t n2)
    {
        return mutator_->addElement(
            std::make_unique<TriangleElement>(std::array<size_t, 3>{n0, n1, n2}));
    }

    MeshData2D meshData_;
    std::unique_ptr<MeshMutator2D> mutator_;
};

TEST_F(Computer2DTest, ComputeEdgeLengthReturnsCorrectDistance)
{
    Point2D p1(0.0, 0.0);
    Point2D p2(3.0, 4.0);

    double length = Computer2D::computeEdgeLength(p1, p2);

    EXPECT_NEAR(length, 5.0, TOLERANCE);
}

TEST_F(Computer2DTest, ComputeEdgeLengthHandlesZeroLength)
{
    Point2D p1(2.0, 3.0);
    Point2D p2(2.0, 3.0);

    double length = Computer2D::computeEdgeLength(p1, p2);

    EXPECT_NEAR(length, 0.0, TOLERANCE);
}

TEST_F(Computer2DTest, ComputeShortestEdgeLengthForEquilateralTriangle)
{
    size_t n0 = addNode(0.0, 0.0);
    size_t n1 = addNode(1.0, 0.0);
    size_t n2 = addNode(0.5, std::sqrt(3.0) / 2.0);
    addTriangle(n0, n1, n2);

    Computer2D computer(meshData_);
    const auto* element = dynamic_cast<const TriangleElement*>(meshData_.getElement(0));

    double shortest = computer.computeShortestEdgeLength(*element);

    EXPECT_NEAR(shortest, 1.0, TOLERANCE);
}

TEST_F(Computer2DTest, ComputeLongestEdgeLengthForRightTriangle)
{
    size_t n0 = addNode(0.0, 0.0);
    size_t n1 = addNode(3.0, 0.0);
    size_t n2 = addNode(0.0, 4.0);
    addTriangle(n0, n1, n2);

    Computer2D computer(meshData_);
    const auto* element = dynamic_cast<const TriangleElement*>(meshData_.getElement(0));

    double longest = computer.computeLongestEdgeLength(*element);

    EXPECT_NEAR(longest, 5.0, TOLERANCE);
}

TEST_F(Computer2DTest, ComputeCircumradiusToShortestEdgeRatioForEquilateral)
{
    size_t n0 = addNode(0.0, 0.0);
    size_t n1 = addNode(1.0, 0.0);
    size_t n2 = addNode(0.5, std::sqrt(3.0) / 2.0);
    addTriangle(n0, n1, n2);

    Computer2D computer(meshData_);
    const auto* element = dynamic_cast<const TriangleElement*>(meshData_.getElement(0));

    auto ratio = computer.computeCircumradiusToShortestEdgeRatio(*element);

    ASSERT_TRUE(ratio.has_value());
    EXPECT_NEAR(ratio.value(), 1.0 / std::sqrt(3.0), TOLERANCE);
}

TEST_F(Computer2DTest, ComputeTriangleAnglesForRightTriangle)
{
    size_t n0 = addNode(0.0, 0.0);
    size_t n1 = addNode(1.0, 0.0);
    size_t n2 = addNode(0.0, 1.0);
    addTriangle(n0, n1, n2);

    Computer2D computer(meshData_);
    const auto* element = dynamic_cast<const TriangleElement*>(meshData_.getElement(0));

    auto angles = computer.computeTriangleAngles(*element);

    EXPECT_NEAR(angles[0], M_PI / 2.0, TOLERANCE);
    EXPECT_NEAR(angles[1], M_PI / 4.0, TOLERANCE);
    EXPECT_NEAR(angles[2], M_PI / 4.0, TOLERANCE);
}

TEST_F(Computer2DTest, ComputeTriangleAnglesForEquilateral)
{
    size_t n0 = addNode(0.0, 0.0);
    size_t n1 = addNode(1.0, 0.0);
    size_t n2 = addNode(0.5, std::sqrt(3.0) / 2.0);
    addTriangle(n0, n1, n2);

    Computer2D computer(meshData_);
    const auto* element = dynamic_cast<const TriangleElement*>(meshData_.getElement(0));

    auto angles = computer.computeTriangleAngles(*element);

    EXPECT_NEAR(angles[0], M_PI / 3.0, TOLERANCE);
    EXPECT_NEAR(angles[1], M_PI / 3.0, TOLERANCE);
    EXPECT_NEAR(angles[2], M_PI / 3.0, TOLERANCE);
}

TEST_F(Computer2DTest, ComputeMinAngleForSkinnyTriangle)
{
    size_t n0 = addNode(0.0, 0.0);
    size_t n1 = addNode(10.0, 0.0);
    size_t n2 = addNode(5.0, 0.5);
    addTriangle(n0, n1, n2);

    Computer2D computer(meshData_);
    const auto* element = dynamic_cast<const TriangleElement*>(meshData_.getElement(0));

    double minAngle = computer.computeMinAngle(*element);

    EXPECT_LT(minAngle, M_PI / 6.0);
}

TEST_F(Computer2DTest, CreateDiametralCircleFromSegment)
{
    Point2D p1(0.0, 0.0);
    Point2D p2(4.0, 0.0);

    DiametralCircle2D circle = Computer2D::createDiametralCircle(p1, p2);

    EXPECT_NEAR(circle.center.x(), 2.0, TOLERANCE);
    EXPECT_NEAR(circle.center.y(), 0.0, TOLERANCE);
    EXPECT_NEAR(circle.radius, 2.0, TOLERANCE);
}

TEST_F(Computer2DTest, IsPointInDiametralCircleDetectsEncroachment)
{
    Point2D p1(0.0, 0.0);
    Point2D p2(4.0, 0.0);
    DiametralCircle2D circle = Computer2D::createDiametralCircle(p1, p2);

    Point2D inside(2.0, 0.5);
    Point2D outside(2.0, 3.0);
    Point2D onCircle(2.0, 2.0);

    EXPECT_TRUE(Computer2D::isPointInDiametralCircle(circle, inside));
    EXPECT_FALSE(Computer2D::isPointInDiametralCircle(circle, outside));
    EXPECT_FALSE(Computer2D::isPointInDiametralCircle(circle, onCircle));
}

TEST_F(Computer2DTest, IsPointInDiametralCircleEndpointsAreOnBoundary)
{
    Point2D p1(0.0, 0.0);
    Point2D p2(4.0, 0.0);
    DiametralCircle2D circle = Computer2D::createDiametralCircle(p1, p2);

    EXPECT_FALSE(Computer2D::isPointInDiametralCircle(circle, p1));
    EXPECT_FALSE(Computer2D::isPointInDiametralCircle(circle, p2));
}
