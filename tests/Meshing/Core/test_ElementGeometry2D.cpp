#include <gtest/gtest.h>

#include "Common/Types.h"
#include "Meshing/Core/2D/General/ElementGeometry2D.h"
#include "Meshing/Core/2D/General/GeometryStructures2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/MeshMutator2D.h"
#include "Meshing/Data/Base/TriangleElement.h"

#include <cmath>

using namespace Meshing;

namespace
{
constexpr double TOLERANCE = 1e-9;
}

class ElementGeometry2DTest : public ::testing::Test
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

TEST_F(ElementGeometry2DTest, ComputeCircumcircleForRightTriangle)
{
    NodeId n0 = addNode(0.0, 0.0);
    NodeId n1 = addNode(1.0, 0.0);
    NodeId n2 = addNode(0.0, 1.0);
    addTriangle(n0, n1, n2);

    ElementGeometry2D geometry(meshData_);
    const auto* element = dynamic_cast<const TriangleElement*>(meshData_.getElement(0));

    auto circle = geometry.computeCircumcircle(*element);

    ASSERT_TRUE(circle.has_value());
    EXPECT_NEAR(circle->center.x(), 0.5, TOLERANCE);
    EXPECT_NEAR(circle->center.y(), 0.5, TOLERANCE);
    EXPECT_NEAR(circle->radius, std::sqrt(2.0) / 2.0, TOLERANCE);
}

TEST_F(ElementGeometry2DTest, ComputeCircumcenterReturnsCorrectPoint)
{
    NodeId n0 = addNode(0.0, 0.0);
    NodeId n1 = addNode(1.0, 0.0);
    NodeId n2 = addNode(0.0, 1.0);
    addTriangle(n0, n1, n2);

    ElementGeometry2D geometry(meshData_);
    const auto* element = dynamic_cast<const TriangleElement*>(meshData_.getElement(0));

    auto circumcenter = geometry.computeCircumcenter(*element);

    ASSERT_TRUE(circumcenter.has_value());
    EXPECT_NEAR(circumcenter->x(), 0.5, TOLERANCE);
    EXPECT_NEAR(circumcenter->y(), 0.5, TOLERANCE);
}

TEST_F(ElementGeometry2DTest, ComputeAreaForRightTriangle)
{
    NodeId n0 = addNode(0.0, 0.0);
    NodeId n1 = addNode(3.0, 0.0);
    NodeId n2 = addNode(0.0, 4.0);
    addTriangle(n0, n1, n2);

    ElementGeometry2D geometry(meshData_);
    const auto* element = dynamic_cast<const TriangleElement*>(meshData_.getElement(0));

    double area = geometry.computeArea(*element);

    EXPECT_NEAR(area, 6.0, TOLERANCE);
}

TEST_F(ElementGeometry2DTest, ComputeAreaForEquilateralTriangle)
{
    NodeId n0 = addNode(0.0, 0.0);
    NodeId n1 = addNode(1.0, 0.0);
    NodeId n2 = addNode(0.5, std::sqrt(3.0) / 2.0);
    addTriangle(n0, n1, n2);

    ElementGeometry2D geometry(meshData_);
    const auto* element = dynamic_cast<const TriangleElement*>(meshData_.getElement(0));

    double area = geometry.computeArea(*element);

    EXPECT_NEAR(area, std::sqrt(3.0) / 4.0, TOLERANCE);
}

TEST_F(ElementGeometry2DTest, ComputeTriangleAnglesForRightTriangle)
{
    NodeId n0 = addNode(0.0, 0.0);
    NodeId n1 = addNode(1.0, 0.0);
    NodeId n2 = addNode(0.0, 1.0);
    addTriangle(n0, n1, n2);

    ElementGeometry2D geometry(meshData_);
    const auto* element = dynamic_cast<const TriangleElement*>(meshData_.getElement(0));

    auto angles = geometry.computeTriangleAngles(*element);

    EXPECT_NEAR(angles[0], M_PI / 2.0, TOLERANCE);
    EXPECT_NEAR(angles[1], M_PI / 4.0, TOLERANCE);
    EXPECT_NEAR(angles[2], M_PI / 4.0, TOLERANCE);
}

TEST_F(ElementGeometry2DTest, ComputeTriangleAnglesForEquilateral)
{
    NodeId n0 = addNode(0.0, 0.0);
    NodeId n1 = addNode(1.0, 0.0);
    NodeId n2 = addNode(0.5, std::sqrt(3.0) / 2.0);
    addTriangle(n0, n1, n2);

    ElementGeometry2D geometry(meshData_);
    const auto* element = dynamic_cast<const TriangleElement*>(meshData_.getElement(0));

    auto angles = geometry.computeTriangleAngles(*element);

    EXPECT_NEAR(angles[0], M_PI / 3.0, TOLERANCE);
    EXPECT_NEAR(angles[1], M_PI / 3.0, TOLERANCE);
    EXPECT_NEAR(angles[2], M_PI / 3.0, TOLERANCE);
}

TEST_F(ElementGeometry2DTest, ComputeMinAngleForSkinnyTriangle)
{
    NodeId n0 = addNode(0.0, 0.0);
    NodeId n1 = addNode(10.0, 0.0);
    NodeId n2 = addNode(5.0, 0.5);
    addTriangle(n0, n1, n2);

    ElementGeometry2D geometry(meshData_);
    const auto* element = dynamic_cast<const TriangleElement*>(meshData_.getElement(0));

    double minAngle = geometry.computeMinAngle(*element);

    EXPECT_LT(minAngle, M_PI / 6.0);
}

TEST_F(ElementGeometry2DTest, ComputeMinAngleForEquilateral)
{
    NodeId n0 = addNode(0.0, 0.0);
    NodeId n1 = addNode(1.0, 0.0);
    NodeId n2 = addNode(0.5, std::sqrt(3.0) / 2.0);
    addTriangle(n0, n1, n2);

    ElementGeometry2D geometry(meshData_);
    const auto* element = dynamic_cast<const TriangleElement*>(meshData_.getElement(0));

    double minAngle = geometry.computeMinAngle(*element);

    EXPECT_NEAR(minAngle, M_PI / 3.0, TOLERANCE);
}
