#include <gtest/gtest.h>

#include "Common/Types.h"
#include "Meshing/Core/3D/General/ElementGeometry3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Meshing/Data/3D/TetrahedronElement.h"
#include "Meshing/Data/Base/TriangleElement.h"

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

    NodeId addNode(double x, double y, double z)
    {
        return mutator_->addNode(Point3D(x, y, z));
    }

    ElementId addTetrahedron(NodeId n0, NodeId n1, NodeId n2, NodeId n3)
    {
        return mutator_->addElement(
            std::make_unique<TetrahedronElement>(std::array<NodeId, 4>{n0, n1, n2, n3}));
    }

    ElementId addTriangle(NodeId n0, NodeId n1, NodeId n2)
    {
        return mutator_->addElement(
            std::make_unique<TriangleElement>(std::array<NodeId, 3>{n0, n1, n2}));
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
    NodeId n0 = addNode(0.0, 0.0, 0.0);
    NodeId n1 = addNode(2.0, 0.0, 0.0);
    NodeId n2 = addNode(0.3, 1.7, 0.0);
    NodeId n3 = addNode(0.4, 0.5, 1.9);
    addTetrahedron(n0, n1, n2, n3);

    const ElementGeometry3D geometry(meshData_);
    const auto* element = dynamic_cast<const TetrahedronElement*>(meshData_.getElement(ElementId{0}));
    const auto orthocenter = geometry.computeOrthocenter(*element);

    ASSERT_TRUE(orthocenter.has_value());
    const double radius = (*orthocenter - meshData_.getNode(n0)->getCoordinates()).norm();
    for (const NodeId nodeId : {n1, n2, n3})
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
    std::array<NodeId, 4> nodeIds;
    for (size_t i = 0; i < 4; ++i)
        nodeIds[i] = mutator_->addNode(points[i], weights[i]);
    addTetrahedron(nodeIds[0], nodeIds[1], nodeIds[2], nodeIds[3]);

    const ElementGeometry3D geometry(meshData_);
    const auto* element = dynamic_cast<const TetrahedronElement*>(meshData_.getElement(ElementId{0}));
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
    std::array<NodeId, 3> nodeIds;
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
// and cocircular, so every point on the line normal to their plane through
// the centre of their circle (0, -0.34, 0) has equal power to all four.
// Exactly flat, the tetrahedron must still get a dual vertex, the one in the
// plane: returning none left the faces around such a tetrahedron without a
// dual edge, never restricted, and the chamfer with holes (OPE-186).
TEST_F(ElementGeometry3DTest, OrthocenterOfExactlyFlatCyclicTetrahedronLiesInItsPlane)
{
    const std::array<Point3D, 4> points = {Point3D(-2.0, -1.0, 0.0), Point3D(2.0, -1.0, 0.0), Point3D(1.0, 1.5, 0.0),
                                           Point3D(-1.0, 1.5, 0.0)};
    const std::array<double, 4> weights = {0.1, 0.1, 0.05, 0.05};
    std::array<NodeId, 4> nodeIds;
    for (size_t i = 0; i < 4; ++i)
        nodeIds[i] = mutator_->addNode(points[i], weights[i]);
    addTetrahedron(nodeIds[0], nodeIds[1], nodeIds[2], nodeIds[3]);

    const ElementGeometry3D geometry(meshData_);
    const auto* element = dynamic_cast<const TetrahedronElement*>(meshData_.getElement(ElementId{0}));
    const auto orthocenter = geometry.computeOrthocenter(*element);
    ASSERT_TRUE(orthocenter.has_value());

    EXPECT_NEAR((*orthocenter - Point3D(0.0, -0.34, 0.0)).norm(), 0.0, TOLERANCE);
}

// Tilted, the same trapezoid is flat only to rounding: in exact arithmetic its
// vertices span a genuine tetrahedron, whose orthocenter is one point on the
// line of equal power, possibly far from the plane. It must still get that
// dual vertex.
TEST_F(ElementGeometry3DTest, OrthocenterOfNearlyFlatCyclicTetrahedronLiesOnItsLineOfEqualPower)
{
    const Eigen::Matrix3d tilt =
        (Eigen::AngleAxisd(0.7, Point3D(1.0, 2.0, 0.5).normalized())).toRotationMatrix();
    const std::array<Point3D, 4> points = {tilt * Point3D(-2.0, -1.0, 0.0), tilt * Point3D(2.0, -1.0, 0.0),
                                           tilt * Point3D(1.0, 1.5, 0.0), tilt * Point3D(-1.0, 1.5, 0.0)};
    const std::array<double, 4> weights = {0.1, 0.1, 0.05, 0.05};
    std::array<NodeId, 4> nodeIds;
    for (size_t i = 0; i < 4; ++i)
        nodeIds[i] = mutator_->addNode(points[i], weights[i]);
    addTetrahedron(nodeIds[0], nodeIds[1], nodeIds[2], nodeIds[3]);

    const ElementGeometry3D geometry(meshData_);
    const auto* element = dynamic_cast<const TetrahedronElement*>(meshData_.getElement(ElementId{0}));
    const auto orthocenter = geometry.computeOrthocenter(*element);
    ASSERT_TRUE(orthocenter.has_value());

    const Point3D normal = tilt * Point3D(0.0, 0.0, 1.0);
    const Point3D circleCenter = tilt * Point3D(0.0, -0.34, 0.0);
    EXPECT_NEAR((*orthocenter - circleCenter).cross(normal).norm(), 0.0, TOLERANCE);
    const double power0 = (*orthocenter - points[0]).squaredNorm() - weights[0];
    for (size_t i = 1; i < 4; ++i)
        EXPECT_NEAR((*orthocenter - points[i]).squaredNorm() - weights[i], power0, TOLERANCE);
}

// OPE-214: a sliver lying in the planar face y = 10 of BoxWithHoleSurface --
// two protecting-ball centres on its edge and two refinement points 7e-11 off
// the plane, nearly cocircular. Its orthocenter lies 4.8e-5 above the plane;
// a floating-point solve put it exactly in the plane, which restricted all
// four of its faces and overlapped them in the surface mesh.
TEST_F(ElementGeometry3DTest, OrthocenterOfSliverInAPlaneIsOnTheCorrectSideOfIt)
{
    const std::array<Point3D, 4> points = {Point3D(10.0, 10.0, 3.333333333333333),
                                           Point3D(6.6216203339892381, 9.9999999999267501, 3.0019632464859658),
                                           Point3D(10.0, 10.0, 6.6666666666666661),
                                           Point3D(6.6216203339892363, 10.000000000073236, 6.998036753514036)};
    const std::array<double, 4> weights = {3.7777777777777795, 0.0, 3.7777777777777795, 0.0};
    std::array<NodeId, 4> nodeIds;
    for (size_t i = 0; i < 4; ++i)
        nodeIds[i] = mutator_->addNode(points[i], weights[i]);
    addTetrahedron(nodeIds[0], nodeIds[1], nodeIds[2], nodeIds[3]);

    const ElementGeometry3D geometry(meshData_);
    const auto* element = dynamic_cast<const TetrahedronElement*>(meshData_.getElement(ElementId{0}));
    const auto orthocenter = geometry.computeOrthocenter(*element);
    ASSERT_TRUE(orthocenter.has_value());

    // 10.000047868244517 in exact rational arithmetic.
    EXPECT_NEAR(orthocenter->y(), 10.000047868244517, 1e-12);
}
