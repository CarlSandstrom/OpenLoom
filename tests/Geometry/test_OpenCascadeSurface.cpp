#include <gtest/gtest.h>

#include "Geometry/3D/OpenCascade/OpenCascadeSurface.h"

#include <BRepBuilderAPI_MakeFace.hxx>
#include <TopoDS.hxx>
#include <cmath>
#include <gp_Ax3.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pln.hxx>
#include <gp_Sphere.hxx>
#include <numbers>

namespace
{

constexpr double TOLERANCE = 1e-9;

TopoDS_Face makePlanarFace()
{
    return BRepBuilderAPI_MakeFace(gp_Pln(gp_Ax3()), -1.0, 1.0, -1.0, 1.0).Face();
}

TopoDS_Face makeCylindricalFace(double radius)
{
    return BRepBuilderAPI_MakeFace(gp_Cylinder(gp_Ax3(), radius),
                                   0.0,
                                   2.0 * std::numbers::pi,
                                   0.0,
                                   1.0)
        .Face();
}

TopoDS_Face makeSphericalFace(double radius)
{
    return BRepBuilderAPI_MakeFace(gp_Sphere(gp_Ax3(), radius),
                                   0.0,
                                   2.0 * std::numbers::pi,
                                   -std::numbers::pi / 2.0,
                                   std::numbers::pi / 2.0)
        .Face();
}

} // namespace

TEST(OpenCascadeSurfaceCurvatureTest, PlaneHasZeroCurvatureInBothDirections)
{
    Geometry3D::OpenCascadeSurface surface(makePlanarFace());

    const auto curvatures = surface.getPrincipalCurvatures(0.25, -0.5);

    ASSERT_TRUE(curvatures.has_value());
    EXPECT_NEAR(curvatures->minimum, 0.0, TOLERANCE);
    EXPECT_NEAR(curvatures->maximum, 0.0, TOLERANCE);
}

TEST(OpenCascadeSurfaceCurvatureTest, CylinderIsFlatAlongItsAxisAndCurvedAround)
{
    constexpr double RADIUS = 2.5;
    Geometry3D::OpenCascadeSurface surface(makeCylindricalFace(RADIUS));

    const auto curvatures = surface.getPrincipalCurvatures(1.0, 0.5);

    ASSERT_TRUE(curvatures.has_value());
    // One principal direction runs along the axis (straight), the other
    // around the circumference (radius of curvature = the cylinder radius).
    EXPECT_NEAR(curvatures->minimum, -1.0 / RADIUS, TOLERANCE);
    EXPECT_NEAR(curvatures->maximum, 0.0, TOLERANCE);
}

TEST(OpenCascadeSurfaceCurvatureTest, SphereCurvesEquallyInEveryDirection)
{
    constexpr double RADIUS = 4.0;
    Geometry3D::OpenCascadeSurface surface(makeSphericalFace(RADIUS));

    const auto curvatures = surface.getPrincipalCurvatures(1.0, 0.0);

    ASSERT_TRUE(curvatures.has_value());
    EXPECT_NEAR(curvatures->minimum, -1.0 / RADIUS, TOLERANCE);
    EXPECT_NEAR(curvatures->maximum, -1.0 / RADIUS, TOLERANCE);
}

TEST(OpenCascadeSurfaceCurvatureTest, ReversedFaceNegatesCurvatureAlongWithItsNormal)
{
    constexpr double RADIUS = 2.5;
    const TopoDS_Face face = makeCylindricalFace(RADIUS);

    Geometry3D::OpenCascadeSurface forward(face);
    Geometry3D::OpenCascadeSurface reversed(TopoDS::Face(face.Reversed()));

    const auto forwardCurvatures = forward.getPrincipalCurvatures(1.0, 0.5);
    const auto reversedCurvatures = reversed.getPrincipalCurvatures(1.0, 0.5);

    ASSERT_TRUE(forwardCurvatures.has_value());
    ASSERT_TRUE(reversedCurvatures.has_value());

    // Curvature is signed against getNormal(), so reversing the face must
    // flip both values -- and swap which one is the minimum.
    EXPECT_NEAR(reversedCurvatures->minimum, -forwardCurvatures->maximum, TOLERANCE);
    EXPECT_NEAR(reversedCurvatures->maximum, -forwardCurvatures->minimum, TOLERANCE);
    EXPECT_LE(reversedCurvatures->minimum, reversedCurvatures->maximum);

    const auto forwardNormal = forward.getNormal(1.0, 0.5);
    const auto reversedNormal = reversed.getNormal(1.0, 0.5);
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        EXPECT_NEAR(reversedNormal[axis], -forwardNormal[axis], TOLERANCE);
    }
}
