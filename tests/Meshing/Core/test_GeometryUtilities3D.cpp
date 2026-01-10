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
