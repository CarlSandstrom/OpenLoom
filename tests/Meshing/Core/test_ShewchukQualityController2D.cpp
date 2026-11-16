#include <gtest/gtest.h>

#include "Common/Types.h"
#include "Meshing/Core/2D/ShewchukQualityController2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/MeshMutator2D.h"
#include "Meshing/Data/Base/TriangleElement.h"

#include <cmath>

using namespace Meshing;

class ShewchukQualityController2DTest : public ::testing::Test
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

TEST_F(ShewchukQualityController2DTest, AcceptsEquilateralTriangle)
{
    size_t n0 = addNode(0.0, 0.0);
    size_t n1 = addNode(1.0, 0.0);
    size_t n2 = addNode(0.5, std::sqrt(3.0) / 2.0);
    addTriangle(n0, n1, n2);

    ShewchukQualityController2D controller(meshData_, std::sqrt(2.0), M_PI / 9.0, 100000);

    const auto* element = dynamic_cast<const TriangleElement*>(meshData_.getElement(0));
    EXPECT_TRUE(controller.isTriangleAcceptable(*element));
}

TEST_F(ShewchukQualityController2DTest, RejectsSkinnyTriangle)
{
    size_t n0 = addNode(0.0, 0.0);
    size_t n1 = addNode(10.0, 0.0);
    size_t n2 = addNode(5.0, 0.1);
    addTriangle(n0, n1, n2);

    ShewchukQualityController2D controller(meshData_, std::sqrt(2.0), M_PI / 9.0, 100000);

    const auto* element = dynamic_cast<const TriangleElement*>(meshData_.getElement(0));
    EXPECT_FALSE(controller.isTriangleAcceptable(*element));
}

TEST_F(ShewchukQualityController2DTest, AcceptsRightTriangle)
{
    size_t n0 = addNode(0.0, 0.0);
    size_t n1 = addNode(1.0, 0.0);
    size_t n2 = addNode(0.0, 1.0);
    addTriangle(n0, n1, n2);

    ShewchukQualityController2D controller(meshData_, std::sqrt(2.0), M_PI / 9.0, 100000);

    const auto* element = dynamic_cast<const TriangleElement*>(meshData_.getElement(0));
    EXPECT_TRUE(controller.isTriangleAcceptable(*element));
}

TEST_F(ShewchukQualityController2DTest, IsMeshAcceptableWithGoodTriangles)
{
    size_t n0 = addNode(0.0, 0.0);
    size_t n1 = addNode(1.0, 0.0);
    size_t n2 = addNode(0.5, std::sqrt(3.0) / 2.0);
    size_t n3 = addNode(1.5, std::sqrt(3.0) / 2.0);
    addTriangle(n0, n1, n2);
    addTriangle(n1, n3, n2);

    ShewchukQualityController2D controller(meshData_, std::sqrt(2.0), M_PI / 9.0, 100000);

    EXPECT_TRUE(controller.isMeshAcceptable(meshData_));
}

TEST_F(ShewchukQualityController2DTest, IsMeshAcceptableRejectsWithBadTriangle)
{
    size_t n0 = addNode(0.0, 0.0);
    size_t n1 = addNode(1.0, 0.0);
    size_t n2 = addNode(0.5, std::sqrt(3.0) / 2.0);
    addTriangle(n0, n1, n2);

    size_t n3 = addNode(10.0, 0.0);
    size_t n4 = addNode(20.0, 0.0);
    size_t n5 = addNode(15.0, 0.1);
    addTriangle(n3, n4, n5);

    ShewchukQualityController2D controller(meshData_, std::sqrt(2.0), M_PI / 9.0, 100000);

    EXPECT_FALSE(controller.isMeshAcceptable(meshData_));
}

TEST_F(ShewchukQualityController2DTest, GetTargetElementQualityReturnsRatioBound)
{
    double ratioBound = 1.5;
    ShewchukQualityController2D controller(meshData_, ratioBound, M_PI / 9.0, 100000);

    EXPECT_DOUBLE_EQ(controller.getTargetElementQuality(), ratioBound);
}

TEST_F(ShewchukQualityController2DTest, GetElementLimitReturnsConfiguredLimit)
{
    std::size_t limit = 50000;
    ShewchukQualityController2D controller(meshData_, std::sqrt(2.0), M_PI / 9.0, limit);

    EXPECT_EQ(controller.getElementLimit(), limit);
}

TEST_F(ShewchukQualityController2DTest, StricterAngleThresholdRejectsMoreTriangles)
{
    size_t n0 = addNode(0.0, 0.0);
    size_t n1 = addNode(1.0, 0.0);
    size_t n2 = addNode(0.0, 1.0);
    addTriangle(n0, n1, n2);

    const auto* element = dynamic_cast<const TriangleElement*>(meshData_.getElement(0));

    ShewchukQualityController2D lenientController(meshData_, std::sqrt(2.0), M_PI / 9.0, 100000);
    ShewchukQualityController2D strictController(meshData_, std::sqrt(2.0), M_PI / 3.0, 100000);

    EXPECT_TRUE(lenientController.isTriangleAcceptable(*element));
    EXPECT_FALSE(strictController.isTriangleAcceptable(*element));
}
