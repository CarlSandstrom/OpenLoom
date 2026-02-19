#include <gtest/gtest.h>

#include "Common/Types.h"
#include "Meshing/Core/3D/General/ElementGeometry3D.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"

#include <cmath>

using namespace Meshing;

namespace
{
constexpr double TOLERANCE = 1e-9;
}

class ElementGeometry3DTest : public ::testing::Test
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

    size_t addTriangle(size_t n0, size_t n1, size_t n2)
    {
        return mutator_->addElement(
            std::make_unique<TriangleElement>(std::array<size_t, 3>{n0, n1, n2}));
    }

    MeshData3D meshData_;
    std::unique_ptr<MeshMutator3D> mutator_;
};

TEST_F(ElementGeometry3DTest, ComputeVolumeForUnitTetrahedron)
{
    // Unit tetrahedron with vertices at (0,0,0), (1,0,0), (0,1,0), (0,0,1)
    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(1.0, 0.0, 0.0);
    size_t n2 = addNode(0.0, 1.0, 0.0);
    size_t n3 = addNode(0.0, 0.0, 1.0);
    addTetrahedron(n0, n1, n2, n3);

    ElementGeometry3D geometry(meshData_);
    const auto* element = dynamic_cast<const TetrahedralElement*>(meshData_.getElement(0));

    double volume = geometry.computeVolume(*element);

    EXPECT_NEAR(volume, 1.0 / 6.0, TOLERANCE);
}

TEST_F(ElementGeometry3DTest, ComputeAreaForTriangleIn3D)
{
    // Right triangle in 3D with legs of length 3 and 4
    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(3.0, 0.0, 0.0);
    size_t n2 = addNode(0.0, 4.0, 0.0);
    addTriangle(n0, n1, n2);

    ElementGeometry3D geometry(meshData_);
    const auto* element = dynamic_cast<const TriangleElement*>(meshData_.getElement(0));

    double area = geometry.computeArea(*element);

    EXPECT_NEAR(area, 6.0, TOLERANCE);
}

TEST_F(ElementGeometry3DTest, ComputeCircumscribingSphereForRegularTetrahedron)
{
    // Regular tetrahedron
    const double a = 1.0;
    const double h = std::sqrt(2.0 / 3.0) * a;

    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(a, 0.0, 0.0);
    size_t n2 = addNode(a / 2.0, a * std::sqrt(3.0) / 2.0, 0.0);
    size_t n3 = addNode(a / 2.0, a * std::sqrt(3.0) / 6.0, h);
    addTetrahedron(n0, n1, n2, n3);

    ElementGeometry3D geometry(meshData_);
    const auto* element = dynamic_cast<const TetrahedralElement*>(meshData_.getElement(0));

    auto sphere = geometry.computeCircumscribingSphere(*element);

    ASSERT_TRUE(sphere.has_value());
    // Regular tetrahedron has circumradius = edge_length * sqrt(6) / 4
    EXPECT_NEAR(sphere->radius, a * std::sqrt(6.0) / 4.0, TOLERANCE);
}

TEST_F(ElementGeometry3DTest, IsPointInsideCircumscribingSphereDetectsContainment)
{
    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(1.0, 0.0, 0.0);
    size_t n2 = addNode(0.0, 1.0, 0.0);
    size_t n3 = addNode(0.0, 0.0, 1.0);
    addTetrahedron(n0, n1, n2, n3);

    ElementGeometry3D geometry(meshData_);
    const auto* element = dynamic_cast<const TetrahedralElement*>(meshData_.getElement(0));

    Point3D inside(0.25, 0.25, 0.25);
    Point3D outside(2.0, 2.0, 2.0);

    EXPECT_TRUE(geometry.isPointInsideCircumscribingSphere(*element, inside));
    EXPECT_FALSE(geometry.isPointInsideCircumscribingSphere(*element, outside));
}

TEST_F(ElementGeometry3DTest, ComputeCentroidForUnitTetrahedron)
{
    // Unit tetrahedron with vertices at (0,0,0), (1,0,0), (0,1,0), (0,0,1)
    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(1.0, 0.0, 0.0);
    size_t n2 = addNode(0.0, 1.0, 0.0);
    size_t n3 = addNode(0.0, 0.0, 1.0);
    addTetrahedron(n0, n1, n2, n3);

    ElementGeometry3D geometry(meshData_);
    const auto* element = dynamic_cast<const TetrahedralElement*>(meshData_.getElement(0));

    Point3D centroid = geometry.computeCentroid(*element);

    // Centroid is average of vertices: (0+1+0+0)/4, (0+0+1+0)/4, (0+0+0+1)/4 = (0.25, 0.25, 0.25)
    EXPECT_NEAR(centroid.x(), 0.25, TOLERANCE);
    EXPECT_NEAR(centroid.y(), 0.25, TOLERANCE);
    EXPECT_NEAR(centroid.z(), 0.25, TOLERANCE);
}

TEST_F(ElementGeometry3DTest, ComputeCentroidForTranslatedTetrahedron)
{
    // Tetrahedron translated by (1, 2, 3)
    size_t n0 = addNode(1.0, 2.0, 3.0);
    size_t n1 = addNode(2.0, 2.0, 3.0);
    size_t n2 = addNode(1.0, 3.0, 3.0);
    size_t n3 = addNode(1.0, 2.0, 4.0);
    addTetrahedron(n0, n1, n2, n3);

    ElementGeometry3D geometry(meshData_);
    const auto* element = dynamic_cast<const TetrahedralElement*>(meshData_.getElement(0));

    Point3D centroid = geometry.computeCentroid(*element);

    // Centroid: (1+2+1+1)/4, (2+2+3+2)/4, (3+3+3+4)/4 = (1.25, 2.25, 3.25)
    EXPECT_NEAR(centroid.x(), 1.25, TOLERANCE);
    EXPECT_NEAR(centroid.y(), 2.25, TOLERANCE);
    EXPECT_NEAR(centroid.z(), 3.25, TOLERANCE);
}

TEST_F(ElementGeometry3DTest, ComputeCentroidForCubicTetrahedron)
{
    // Tetrahedron with vertices at corners of a unit cube
    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(1.0, 1.0, 0.0);
    size_t n2 = addNode(1.0, 0.0, 1.0);
    size_t n3 = addNode(0.0, 1.0, 1.0);
    addTetrahedron(n0, n1, n2, n3);

    ElementGeometry3D geometry(meshData_);
    const auto* element = dynamic_cast<const TetrahedralElement*>(meshData_.getElement(0));

    Point3D centroid = geometry.computeCentroid(*element);

    // Centroid: (0+1+1+0)/4, (0+1+0+1)/4, (0+0+1+1)/4 = (0.5, 0.5, 0.5)
    EXPECT_NEAR(centroid.x(), 0.5, TOLERANCE);
    EXPECT_NEAR(centroid.y(), 0.5, TOLERANCE);
    EXPECT_NEAR(centroid.z(), 0.5, TOLERANCE);
}
