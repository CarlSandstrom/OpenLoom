#include <gtest/gtest.h>

#include "Common/Types.h"
#include "Meshing/Core/3D/General/ElementGeometry3D.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"

#include <Eigen/Geometry>
#include <algorithm>
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

// ============================================================================
// computeOrthocenter tests
// ============================================================================

// With no weights a regular triangulation is a Delaunay triangulation, so the
// orthocenter must reduce to the ordinary circumcenter: equidistant from all
// four vertices.
TEST_F(ElementGeometry3DTest, OrthocenterOfUnweightedTetrahedronIsItsCircumcenter)
{
    const size_t n0 = addNode(0.0, 0.0, 0.0);
    const size_t n1 = addNode(2.0, 0.0, 0.0);
    const size_t n2 = addNode(0.3, 1.7, 0.0);
    const size_t n3 = addNode(0.4, 0.5, 1.9);
    addTetrahedron(n0, n1, n2, n3);

    const ElementGeometry3D geometry(meshData_);
    const auto* element = dynamic_cast<const TetrahedralElement*>(meshData_.getElement(0));
    const auto orthocenter = geometry.computeOrthocenter(*element);

    ASSERT_TRUE(orthocenter.has_value());
    const double radius = (*orthocenter - meshData_.getNode(n0)->getCoordinates()).norm();
    for (const size_t nodeId : {n1, n2, n3})
    {
        EXPECT_NEAR((*orthocenter - meshData_.getNode(nodeId)->getCoordinates()).norm(), radius, TOLERANCE);
    }
}

// The defining property: equal power distance |x - p|^2 - w to every vertex.
// Weights here are of the size protecting balls give crease nodes, which is
// what moves the dual edge away from the ordinary circumcenter.
TEST_F(ElementGeometry3DTest, OrthocenterOfWeightedTetrahedronHasEqualPowerToAllVertices)
{
    const std::array<Point3D, 4> points = {Point3D(0.0, 0.0, 0.0), Point3D(2.0, 0.0, 0.0), Point3D(0.3, 1.7, 0.0),
                                           Point3D(0.4, 0.5, 1.9)};
    const std::array<double, 4> weights = {0.25, 0.09, 0.0, 0.16};
    std::array<size_t, 4> nodeIds;
    for (size_t i = 0; i < 4; ++i)
        nodeIds[i] = mutator_->addNode(points[i], weights[i]);
    addTetrahedron(nodeIds[0], nodeIds[1], nodeIds[2], nodeIds[3]);

    const ElementGeometry3D geometry(meshData_);
    const auto* element = dynamic_cast<const TetrahedralElement*>(meshData_.getElement(0));
    const auto orthocenter = geometry.computeOrthocenter(*element);
    ASSERT_TRUE(orthocenter.has_value());

    const double power0 = (*orthocenter - points[0]).squaredNorm() - weights[0];
    for (size_t i = 1; i < 4; ++i)
        EXPECT_NEAR((*orthocenter - points[i]).squaredNorm() - weights[i], power0, TOLERANCE);

    // The circumcenter is the one point equidistant from all four vertices,
    // so unequal distances show the weights moved the orthocenter off it.
    double largestDistanceDifference = 0.0;
    const double distance0 = (*orthocenter - points[0]).norm();
    for (size_t i = 1; i < 4; ++i)
        largestDistanceDifference =
            std::max(largestDistanceDifference, std::abs((*orthocenter - points[i]).norm() - distance0));
    EXPECT_GT(largestDistanceDifference, 0.01);
}

// The triangle version lies in the triangle's plane at equal power to its
// three vertices.
TEST_F(ElementGeometry3DTest, OrthocenterOfWeightedTriangleLiesInPlaneAtEqualPower)
{
    const std::array<Point3D, 3> points = {Point3D(0.0, 0.0, 1.0), Point3D(2.0, 0.5, 1.0), Point3D(0.5, 1.8, 1.0)};
    const std::array<double, 3> weights = {0.2, 0.0, 0.05};
    std::array<size_t, 3> nodeIds;
    for (size_t i = 0; i < 3; ++i)
        nodeIds[i] = mutator_->addNode(points[i], weights[i]);

    const ElementGeometry3D geometry(meshData_);
    const auto orthocenter = geometry.computeOrthocenter(TriangleElement(nodeIds));
    ASSERT_TRUE(orthocenter.has_value());

    EXPECT_NEAR(orthocenter->z(), 1.0, TOLERANCE);
    const double power0 = (*orthocenter - points[0]).squaredNorm() - weights[0];
    for (size_t i = 1; i < 3; ++i)
        EXPECT_NEAR((*orthocenter - points[i]).squaredNorm() - weights[i], power0, TOLERANCE);
}

// Four points of an isosceles trapezoid -- matching samples on the two
// circles bounding a cone, as on HexNutChamfered's chamfers -- are coplanar
// and cocircular, so the tetrahedron they span is flat to rounding and every
// point on the line normal to their plane has equal power to all four. It
// must still get a dual vertex, the one in the plane: returning none left the
// faces around such a tetrahedron without a dual edge, never restricted, and
// the chamfer with holes (OPE-186).
TEST_F(ElementGeometry3DTest, OrthocenterOfFlatCyclicTetrahedronLiesInItsPlane)
{
    const Eigen::Matrix3d tilt =
        (Eigen::AngleAxisd(0.7, Point3D(1.0, 2.0, 0.5).normalized())).toRotationMatrix();
    const std::array<Point3D, 4> points = {tilt * Point3D(-2.0, -1.0, 0.0), tilt * Point3D(2.0, -1.0, 0.0),
                                           tilt * Point3D(1.0, 1.5, 0.0), tilt * Point3D(-1.0, 1.5, 0.0)};
    const std::array<double, 4> weights = {0.1, 0.1, 0.05, 0.05};
    std::array<size_t, 4> nodeIds;
    for (size_t i = 0; i < 4; ++i)
        nodeIds[i] = mutator_->addNode(points[i], weights[i]);
    addTetrahedron(nodeIds[0], nodeIds[1], nodeIds[2], nodeIds[3]);

    const ElementGeometry3D geometry(meshData_);
    const auto* element = dynamic_cast<const TetrahedralElement*>(meshData_.getElement(0));
    const auto orthocenter = geometry.computeOrthocenter(*element);
    ASSERT_TRUE(orthocenter.has_value());

    const Point3D normal = tilt * Point3D(0.0, 0.0, 1.0);
    EXPECT_NEAR((*orthocenter - points[0]).dot(normal), 0.0, TOLERANCE);
    const double power0 = (*orthocenter - points[0]).squaredNorm() - weights[0];
    for (size_t i = 1; i < 4; ++i)
        EXPECT_NEAR((*orthocenter - points[i]).squaredNorm() - weights[i], power0, TOLERANCE);
}
