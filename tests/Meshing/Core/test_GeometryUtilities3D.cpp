#include <gtest/gtest.h>

#include "Common/Types.h"
#include "Meshing/Core/3D/GeometryStructures3D.h"
#include "Meshing/Core/3D/GeometryUtilities3D.h"

using namespace Meshing;

namespace
{
constexpr double TOLERANCE = 1e-9;
}

TEST(GeometryUtilities3DTest, IsPointInsideCircumscribingSphereDetectsContainment)
{
    CircumscribedSphere sphere;
    sphere.center = Point3D(0.0, 0.0, 0.0);
    sphere.radius = 5.0;

    Point3D inside(3.0, 0.0, 0.0);
    Point3D outside(6.0, 0.0, 0.0);

    EXPECT_TRUE(GeometryUtilities3D::isPointInsideCircumscribingSphere(sphere, inside));
    EXPECT_FALSE(GeometryUtilities3D::isPointInsideCircumscribingSphere(sphere, outside));
}

TEST(GeometryUtilities3DTest, IsPointInsideCircumscribingSphereWithTolerance)
{
    CircumscribedSphere sphere;
    sphere.center = Point3D(0.0, 0.0, 0.0);
    sphere.radius = 5.0;

    Point3D onSurface(5.0, 0.0, 0.0);
    Point3D slightlyOutside(5.01, 0.0, 0.0);

    // With default tolerance, point on surface should be inside
    EXPECT_TRUE(GeometryUtilities3D::isPointInsideCircumscribingSphere(sphere, onSurface));

    // Point clearly outside the sphere
    EXPECT_FALSE(GeometryUtilities3D::isPointInsideCircumscribingSphere(sphere, slightlyOutside));

    // With large tolerance, even far outside point should be inside
    EXPECT_TRUE(GeometryUtilities3D::isPointInsideCircumscribingSphere(sphere, slightlyOutside, 1.0));
}

TEST(GeometryUtilities3DTest, IsPointInsideCircumscribingSphereCenterPoint)
{
    CircumscribedSphere sphere;
    sphere.center = Point3D(1.0, 2.0, 3.0);
    sphere.radius = 10.0;

    Point3D center(1.0, 2.0, 3.0);

    EXPECT_TRUE(GeometryUtilities3D::isPointInsideCircumscribingSphere(sphere, center));
}

// ============================================================================
// Edge Length Tests
// ============================================================================

TEST(GeometryUtilities3DTest, ComputeEdgeLengthAxisAligned)
{
    Point3D p1(0.0, 0.0, 0.0);
    Point3D p2(3.0, 0.0, 0.0);

    double length = GeometryUtilities3D::computeEdgeLength(p1, p2);
    EXPECT_NEAR(length, 3.0, TOLERANCE);
}

TEST(GeometryUtilities3DTest, ComputeEdgeLengthDiagonal)
{
    Point3D p1(0.0, 0.0, 0.0);
    Point3D p2(1.0, 1.0, 1.0);

    double length = GeometryUtilities3D::computeEdgeLength(p1, p2);
    EXPECT_NEAR(length, std::sqrt(3.0), TOLERANCE);
}

TEST(GeometryUtilities3DTest, ComputeEdgeLengthZero)
{
    Point3D p1(5.0, 5.0, 5.0);
    Point3D p2(5.0, 5.0, 5.0);

    double length = GeometryUtilities3D::computeEdgeLength(p1, p2);
    EXPECT_NEAR(length, 0.0, TOLERANCE);
}

// ============================================================================
// Diametral Sphere Tests
// ============================================================================

TEST(GeometryUtilities3DTest, CreateDiametralSphereAxisAligned)
{
    Point3D p1(0.0, 0.0, 0.0);
    Point3D p2(4.0, 0.0, 0.0);

    DiametralSphere sphere = GeometryUtilities3D::createDiametralSphere(p1, p2);

    // Center should be midpoint
    EXPECT_NEAR(sphere.center.x(), 2.0, TOLERANCE);
    EXPECT_NEAR(sphere.center.y(), 0.0, TOLERANCE);
    EXPECT_NEAR(sphere.center.z(), 0.0, TOLERANCE);

    // Radius should be half the segment length
    EXPECT_NEAR(sphere.radius, 2.0, TOLERANCE);
}

TEST(GeometryUtilities3DTest, CreateDiametralSphereDiagonal)
{
    Point3D p1(0.0, 0.0, 0.0);
    Point3D p2(2.0, 2.0, 2.0);

    DiametralSphere sphere = GeometryUtilities3D::createDiametralSphere(p1, p2);

    // Center should be midpoint
    EXPECT_NEAR(sphere.center.x(), 1.0, TOLERANCE);
    EXPECT_NEAR(sphere.center.y(), 1.0, TOLERANCE);
    EXPECT_NEAR(sphere.center.z(), 1.0, TOLERANCE);

    // Radius should be half the diagonal length
    double expectedRadius = std::sqrt(12.0) / 2.0;
    EXPECT_NEAR(sphere.radius, expectedRadius, TOLERANCE);
}

TEST(GeometryUtilities3DTest, IsPointInDiametralSphereInside)
{
    Point3D p1(0.0, 0.0, 0.0);
    Point3D p2(4.0, 0.0, 0.0);
    DiametralSphere sphere = GeometryUtilities3D::createDiametralSphere(p1, p2);

    // Point at center
    Point3D center(2.0, 0.0, 0.0);
    EXPECT_TRUE(GeometryUtilities3D::isPointInDiametralSphere(sphere, center));

    // Point slightly off center but inside
    Point3D inside(2.0, 0.5, 0.5);
    EXPECT_TRUE(GeometryUtilities3D::isPointInDiametralSphere(sphere, inside));
}

TEST(GeometryUtilities3DTest, IsPointInDiametralSphereOutside)
{
    Point3D p1(0.0, 0.0, 0.0);
    Point3D p2(4.0, 0.0, 0.0);
    DiametralSphere sphere = GeometryUtilities3D::createDiametralSphere(p1, p2);

    // Point far outside
    Point3D outside(10.0, 10.0, 10.0);
    EXPECT_FALSE(GeometryUtilities3D::isPointInDiametralSphere(sphere, outside));

    // Point just outside (radius is 2, so distance > 2 from center)
    Point3D justOutside(2.0, 2.5, 0.0);
    EXPECT_FALSE(GeometryUtilities3D::isPointInDiametralSphere(sphere, justOutside));
}

TEST(GeometryUtilities3DTest, IsPointInDiametralSphereOnBoundary)
{
    Point3D p1(0.0, 0.0, 0.0);
    Point3D p2(2.0, 0.0, 0.0);
    DiametralSphere sphere = GeometryUtilities3D::createDiametralSphere(p1, p2);

    // Center is at (1,0,0), radius is 1
    // Point on boundary (distance = 1 from center)
    Point3D onBoundary(1.0, 1.0, 0.0);

    // Boundary points should NOT be considered inside (strict inequality)
    EXPECT_FALSE(GeometryUtilities3D::isPointInDiametralSphere(sphere, onBoundary));
}

// ============================================================================
// Equatorial Sphere Tests
// ============================================================================

TEST(GeometryUtilities3DTest, CreateEquatorialSphereRightTriangle)
{
    // Right triangle with legs of length 1
    Point3D p1(0.0, 0.0, 0.0);
    Point3D p2(1.0, 0.0, 0.0);
    Point3D p3(0.0, 1.0, 0.0);

    EquatorialSphere sphere = GeometryUtilities3D::createEquatorialSphere(p1, p2, p3);

    // For a right triangle, circumcenter is at midpoint of hypotenuse
    EXPECT_NEAR(sphere.center.x(), 0.5, TOLERANCE);
    EXPECT_NEAR(sphere.center.y(), 0.5, TOLERANCE);
    EXPECT_NEAR(sphere.center.z(), 0.0, TOLERANCE);

    // Circumradius should be half the hypotenuse
    double hypotenuse = std::sqrt(2.0);
    EXPECT_NEAR(sphere.radius, hypotenuse / 2.0, TOLERANCE);
}

TEST(GeometryUtilities3DTest, CreateEquatorialSphereEquilateralTriangle)
{
    // Equilateral triangle with side length 2
    double h = std::sqrt(3.0);
    Point3D p1(0.0, 0.0, 0.0);
    Point3D p2(2.0, 0.0, 0.0);
    Point3D p3(1.0, h, 0.0);

    EquatorialSphere sphere = GeometryUtilities3D::createEquatorialSphere(p1, p2, p3);

    // Circumcenter should be at centroid for equilateral triangle
    EXPECT_NEAR(sphere.center.x(), 1.0, TOLERANCE);
    EXPECT_NEAR(sphere.center.y(), h / 3.0, TOLERANCE);
    EXPECT_NEAR(sphere.center.z(), 0.0, TOLERANCE);

    // Circumradius for equilateral triangle: R = a / sqrt(3) where a is side length
    double expectedRadius = 2.0 / std::sqrt(3.0);
    EXPECT_NEAR(sphere.radius, expectedRadius, TOLERANCE);
}

TEST(GeometryUtilities3DTest, CreateEquatorialSphereArbitraryPlane)
{
    // Triangle not in XY plane
    Point3D p1(1.0, 0.0, 0.0);
    Point3D p2(0.0, 1.0, 0.0);
    Point3D p3(0.0, 0.0, 1.0);

    EquatorialSphere sphere = GeometryUtilities3D::createEquatorialSphere(p1, p2, p3);

    // Verify circumcenter is equidistant from all three vertices
    double d1 = (sphere.center - p1).norm();
    double d2 = (sphere.center - p2).norm();
    double d3 = (sphere.center - p3).norm();

    EXPECT_NEAR(d1, sphere.radius, TOLERANCE);
    EXPECT_NEAR(d2, sphere.radius, TOLERANCE);
    EXPECT_NEAR(d3, sphere.radius, TOLERANCE);
}

TEST(GeometryUtilities3DTest, CreateEquatorialSphereDegenerateTriangle)
{
    // Collinear points (degenerate triangle)
    Point3D p1(0.0, 0.0, 0.0);
    Point3D p2(1.0, 0.0, 0.0);
    Point3D p3(2.0, 0.0, 0.0);

    EquatorialSphere sphere = GeometryUtilities3D::createEquatorialSphere(p1, p2, p3);

    // Should return a sphere with zero radius at centroid
    EXPECT_NEAR(sphere.radius, 0.0, TOLERANCE);
}

TEST(GeometryUtilities3DTest, IsPointInEquatorialSphereNonCoplanarInside)
{
    Point3D p1(0.0, 0.0, 0.0);
    Point3D p2(2.0, 0.0, 0.0);
    Point3D p3(1.0, 2.0, 0.0);

    EquatorialSphere sphere = GeometryUtilities3D::createEquatorialSphere(p1, p2, p3);

    // Point just above center (non-coplanar, inside sphere)
    Point3D above(1.0, 0.5, 0.1);

    EXPECT_TRUE(GeometryUtilities3D::isPointInEquatorialSphere(
        sphere, above, p1, p2, p3));
}

TEST(GeometryUtilities3DTest, IsPointInEquatorialSphereCoplanarReturnsFalse)
{
    Point3D p1(0.0, 0.0, 0.0);
    Point3D p2(2.0, 0.0, 0.0);
    Point3D p3(1.0, 2.0, 0.0);

    EquatorialSphere sphere = GeometryUtilities3D::createEquatorialSphere(p1, p2, p3);

    // Coplanar point (z = 0)
    Point3D coplanar(1.0, 1.0, 0.0);

    // Coplanar points should NOT be considered encroaching
    EXPECT_FALSE(GeometryUtilities3D::isPointInEquatorialSphere(
        sphere, coplanar, p1, p2, p3));
}

TEST(GeometryUtilities3DTest, IsPointInEquatorialSphereNonCoplanarOutside)
{
    Point3D p1(0.0, 0.0, 0.0);
    Point3D p2(2.0, 0.0, 0.0);
    Point3D p3(1.0, 2.0, 0.0);

    EquatorialSphere sphere = GeometryUtilities3D::createEquatorialSphere(p1, p2, p3);

    // Point far from triangle (non-coplanar but outside sphere)
    Point3D farAway(1.0, 1.0, 10.0);

    EXPECT_FALSE(GeometryUtilities3D::isPointInEquatorialSphere(
        sphere, farAway, p1, p2, p3));
}

TEST(GeometryUtilities3DTest, IsPointInEquatorialSphereBothSidesOfPlane)
{
    Point3D p1(0.0, 0.0, 0.0);
    Point3D p2(2.0, 0.0, 0.0);
    Point3D p3(1.0, 2.0, 0.0);

    EquatorialSphere sphere = GeometryUtilities3D::createEquatorialSphere(p1, p2, p3);

    // Points on both sides of the triangle plane
    Point3D above(1.0, 0.5, 0.1);
    Point3D below(1.0, 0.5, -0.1);

    // Both should be inside if close enough
    EXPECT_TRUE(GeometryUtilities3D::isPointInEquatorialSphere(
        sphere, above, p1, p2, p3));
    EXPECT_TRUE(GeometryUtilities3D::isPointInEquatorialSphere(
        sphere, below, p1, p2, p3));
}

// ============================================================================
// Triangle Centroid Tests
// ============================================================================

TEST(GeometryUtilities3DTest, ComputeTriangleCentroidSimple)
{
    Point3D v1(0.0, 0.0, 0.0);
    Point3D v2(3.0, 0.0, 0.0);
    Point3D v3(0.0, 3.0, 0.0);

    Point3D centroid = GeometryUtilities3D::computeTriangleCentroid(v1, v2, v3);

    EXPECT_NEAR(centroid.x(), 1.0, TOLERANCE);
    EXPECT_NEAR(centroid.y(), 1.0, TOLERANCE);
    EXPECT_NEAR(centroid.z(), 0.0, TOLERANCE);
}

TEST(GeometryUtilities3DTest, ComputeTriangleCentroid3D)
{
    Point3D v1(1.0, 2.0, 3.0);
    Point3D v2(4.0, 5.0, 6.0);
    Point3D v3(7.0, 8.0, 9.0);

    Point3D centroid = GeometryUtilities3D::computeTriangleCentroid(v1, v2, v3);

    EXPECT_NEAR(centroid.x(), 4.0, TOLERANCE);
    EXPECT_NEAR(centroid.y(), 5.0, TOLERANCE);
    EXPECT_NEAR(centroid.z(), 6.0, TOLERANCE);
}

TEST(GeometryUtilities3DTest, ComputePointToTriangleCentroidDistanceAtCentroid)
{
    Point3D v1(0.0, 0.0, 0.0);
    Point3D v2(3.0, 0.0, 0.0);
    Point3D v3(0.0, 3.0, 0.0);

    // Point at centroid
    Point3D point(1.0, 1.0, 0.0);

    double distance = GeometryUtilities3D::computePointToTriangleCentroidDistance(point, v1, v2, v3);

    EXPECT_NEAR(distance, 0.0, TOLERANCE);
}

TEST(GeometryUtilities3DTest, ComputePointToTriangleCentroidDistanceAbove)
{
    Point3D v1(0.0, 0.0, 0.0);
    Point3D v2(3.0, 0.0, 0.0);
    Point3D v3(0.0, 3.0, 0.0);

    // Point above the centroid
    Point3D point(1.0, 1.0, 5.0);

    double distance = GeometryUtilities3D::computePointToTriangleCentroidDistance(point, v1, v2, v3);

    EXPECT_NEAR(distance, 5.0, TOLERANCE);
}

TEST(GeometryUtilities3DTest, ComputePointToTriangleCentroidDistanceOffset)
{
    Point3D v1(0.0, 0.0, 0.0);
    Point3D v2(3.0, 0.0, 0.0);
    Point3D v3(0.0, 3.0, 0.0);
    // Centroid is at (1, 1, 0)

    // Point offset from centroid
    Point3D point(4.0, 1.0, 0.0);

    double distance = GeometryUtilities3D::computePointToTriangleCentroidDistance(point, v1, v2, v3);

    EXPECT_NEAR(distance, 3.0, TOLERANCE); // distance from (1,1,0) to (4,1,0)
}
