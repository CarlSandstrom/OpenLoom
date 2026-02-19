#include <gtest/gtest.h>

#include "Common/Types.h"
#include "Meshing/Core/3D/General/ElementQuality3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"

#include <cmath>

using namespace Meshing;

namespace
{
constexpr double TOLERANCE = 1e-9;
}

class ElementQuality3DTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        mutator_ = std::make_unique<MeshMutator3D>(meshData_);
    }

    size_t addNode(double x, double y, double z)
    {
        return mutator_->addNode(Point3D(x, y, z));
    }

    size_t addTetrahedron(size_t n0, size_t n1, size_t n2, size_t n3)
    {
        return mutator_->addElement(
            std::make_unique<TetrahedralElement>(std::array<size_t, 4>{n0, n1, n2, n3}));
    }

    MeshData3D meshData_;
    std::unique_ptr<MeshMutator3D> mutator_;
};

TEST_F(ElementQuality3DTest, GetShortestEdgeLengthForUnitTetrahedron)
{
    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(1.0, 0.0, 0.0);
    size_t n2 = addNode(0.0, 1.0, 0.0);
    size_t n3 = addNode(0.0, 0.0, 1.0);
    addTetrahedron(n0, n1, n2, n3);

    ElementQuality3D quality(meshData_);
    const auto* element = dynamic_cast<const TetrahedralElement*>(meshData_.getElement(0));

    double shortest = quality.getShortestEdgeLength(*element);

    EXPECT_NEAR(shortest, 1.0, TOLERANCE);
}

TEST_F(ElementQuality3DTest, GetShortestEdgeLengthForIrregularTetrahedron)
{
    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(2.0, 0.0, 0.0);
    size_t n2 = addNode(0.0, 3.0, 0.0);
    size_t n3 = addNode(0.0, 0.0, 1.0);
    addTetrahedron(n0, n1, n2, n3);

    ElementQuality3D quality(meshData_);
    const auto* element = dynamic_cast<const TetrahedralElement*>(meshData_.getElement(0));

    double shortest = quality.getShortestEdgeLength(*element);

    EXPECT_NEAR(shortest, 1.0, TOLERANCE);
}

TEST_F(ElementQuality3DTest, GetCircumradiusToShortestEdgeRatioForRegularTetrahedron)
{
    // Regular tetrahedron with edge length 1
    const double a = 1.0;
    const double h = std::sqrt(2.0 / 3.0) * a;

    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(a, 0.0, 0.0);
    size_t n2 = addNode(a / 2.0, a * std::sqrt(3.0) / 2.0, 0.0);
    size_t n3 = addNode(a / 2.0, a * std::sqrt(3.0) / 6.0, h);
    addTetrahedron(n0, n1, n2, n3);

    ElementQuality3D quality(meshData_);
    const auto* element = dynamic_cast<const TetrahedralElement*>(meshData_.getElement(0));

    double ratio = quality.getCircumradiusToShortestEdgeRatio(*element);

    // Regular tetrahedron has optimal ratio = sqrt(6) / 4 ≈ 0.612
    EXPECT_NEAR(ratio, std::sqrt(6.0) / 4.0, TOLERANCE);
}

TEST_F(ElementQuality3DTest, IsSkinnyDetectsSkinnyTetrahedron)
{
    // Create a very flat (skinny) tetrahedron
    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(10.0, 0.0, 0.0);
    size_t n2 = addNode(5.0, 10.0, 0.0);
    size_t n3 = addNode(5.0, 5.0, 0.1); // Very small height
    addTetrahedron(n0, n1, n2, n3);

    ElementQuality3D quality(meshData_);
    const auto* element = dynamic_cast<const TetrahedralElement*>(meshData_.getElement(0));

    // With a threshold of 1.0, this skinny tetrahedron should be detected
    EXPECT_TRUE(quality.isSkinny(*element, 1.0));

    // With a very high threshold, it should still be detected
    EXPECT_TRUE(quality.isSkinny(*element, 10.0));
}

TEST_F(ElementQuality3DTest, IsSkinnyDoesNotDetectGoodTetrahedron)
{
    // Regular tetrahedron (good quality)
    const double a = 1.0;
    const double h = std::sqrt(2.0 / 3.0) * a;

    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(a, 0.0, 0.0);
    size_t n2 = addNode(a / 2.0, a * std::sqrt(3.0) / 2.0, 0.0);
    size_t n3 = addNode(a / 2.0, a * std::sqrt(3.0) / 6.0, h);
    addTetrahedron(n0, n1, n2, n3);

    ElementQuality3D quality(meshData_);
    const auto* element = dynamic_cast<const TetrahedralElement*>(meshData_.getElement(0));

    // With a threshold of 1.0, this good tetrahedron should not be detected as skinny
    EXPECT_FALSE(quality.isSkinny(*element, 1.0));
}
