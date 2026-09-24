#include <gtest/gtest.h>

#include "Common/Types.h"
#include "Meshing/Core/3D/General/MeshingContext3D.h"
#include "Meshing/Core/3D/RCDT/RCDTTetQualityController.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"
#include "Meshing/Data/3D/TetrahedralElement.h"

#include <memory>

using namespace Meshing;

class RCDTTetQualityControllerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        context_ = std::make_unique<MeshingContext3D>();
    }

    size_t addNode(double x, double y, double z)
    {
        return context_->getMutator().addNode(Point3D(x, y, z));
    }

    size_t addTetrahedron(size_t n0, size_t n1, size_t n2, size_t n3)
    {
        auto tet = std::make_unique<TetrahedralElement>(std::array<size_t, 4>{n0, n1, n2, n3});
        return context_->getMutator().addElement(std::move(tet));
    }

    // Regular tetrahedron, edge length 2 — good circumradius/edge ratio.
    size_t createRegularTetrahedron()
    {
        const size_t n0 = addNode(1.0, 1.0, 1.0);
        const size_t n1 = addNode(-1.0, -1.0, 1.0);
        const size_t n2 = addNode(-1.0, 1.0, -1.0);
        const size_t n3 = addNode(1.0, -1.0, -1.0);
        return addTetrahedron(n0, n1, n2, n3);
    }

    // Very flat tetrahedron — high circumradius/edge ratio.
    size_t createSkinnyTetrahedron()
    {
        const size_t n0 = addNode(0.0, 0.0, 0.0);
        const size_t n1 = addNode(10.0, 0.0, 0.0);
        const size_t n2 = addNode(5.0, 10.0, 0.0);
        const size_t n3 = addNode(5.0, 3.0, 0.1);
        return addTetrahedron(n0, n1, n2, n3);
    }

    std::unique_ptr<MeshingContext3D> context_;
};

TEST_F(RCDTTetQualityControllerTest, AcceptsGoodQualityTetrahedron)
{
    const size_t tetId = createRegularTetrahedron();

    SurfaceMesh3DQualitySettings settings;
    settings.tetCircumradiusToShortestEdgeRatio = 2.5;
    RCDTTetQualityController controller(context_->getMeshData(), settings);

    const auto* tet = dynamic_cast<const TetrahedralElement*>(context_->getMeshData().getElement(tetId));
    ASSERT_NE(tet, nullptr);

    EXPECT_TRUE(controller.isTetrahedronAcceptable(*tet));
}

TEST_F(RCDTTetQualityControllerTest, RejectsSkinnyTetrahedron)
{
    const size_t tetId = createSkinnyTetrahedron();

    SurfaceMesh3DQualitySettings settings;
    settings.tetCircumradiusToShortestEdgeRatio = 2.5;
    RCDTTetQualityController controller(context_->getMeshData(), settings);

    const auto* tet = dynamic_cast<const TetrahedralElement*>(context_->getMeshData().getElement(tetId));
    ASSERT_NE(tet, nullptr);

    EXPECT_FALSE(controller.isTetrahedronAcceptable(*tet));
}

TEST_F(RCDTTetQualityControllerTest, TooSmallTetrahedronIsNotRefinable)
{
    const size_t n0 = addNode(0.0, 0.0, 0.0);
    const size_t n1 = addNode(1e-6, 0.0, 0.0);
    const size_t n2 = addNode(0.0, 1e-6, 0.0);
    const size_t n3 = addNode(0.0, 0.0, 1e-6);
    const size_t tetId = addTetrahedron(n0, n1, n2, n3);

    SurfaceMesh3DQualitySettings settings;
    RCDTTetQualityController controller(context_->getMeshData(), settings);

    const auto* tet = dynamic_cast<const TetrahedralElement*>(context_->getMeshData().getElement(tetId));
    ASSERT_NE(tet, nullptr);

    EXPECT_TRUE(controller.isTetrahedronTooSmall(*tet));
}

TEST_F(RCDTTetQualityControllerTest, RegularTetrahedronIsNotTooSmall)
{
    const size_t tetId = createRegularTetrahedron();

    SurfaceMesh3DQualitySettings settings;
    RCDTTetQualityController controller(context_->getMeshData(), settings);

    const auto* tet = dynamic_cast<const TetrahedralElement*>(context_->getMeshData().getElement(tetId));
    ASSERT_NE(tet, nullptr);

    EXPECT_FALSE(controller.isTetrahedronTooSmall(*tet));
}

TEST_F(RCDTTetQualityControllerTest, ReportsTheConfiguredQualityBoundAsItsTarget)
{
    SurfaceMesh3DQualitySettings settings;
    settings.tetCircumradiusToShortestEdgeRatio = 3.0;
    RCDTTetQualityController controller(context_->getMeshData(), settings);

    EXPECT_DOUBLE_EQ(controller.getTargetElementQuality(), 3.0);
}
