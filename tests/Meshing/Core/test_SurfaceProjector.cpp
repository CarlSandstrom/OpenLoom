#include "Meshing/Core/3D/RCDT/SurfaceProjector.h"

#include "Common/BoundingBox2D.h"
#include "Common/Types.h"
#include "Geometry/3D/Base/ISurface3D.h"

#include <cmath>
#include <gtest/gtest.h>
#include <optional>

using namespace Meshing;

// ============================================================================
// MockCylinderSurface
//
// Cylinder of radius R_ around the Z-axis, height [0, height_].
//   getPoint(u, v)    = (R·cos(u), R·sin(u), v),  u ∈ [0, 2π), v ∈ [0, H]
//   getNormal(u, v)   = outward radial (cos(u), sin(u), 0)
//   getGap(p)         = | sqrt(x²+y²) - R |
//   signedDistance via SurfaceProjector reduces to  r - R
// ============================================================================

namespace
{

class MockCylinderSurface : public Geometry3D::ISurface3D
{
public:
    MockCylinderSurface(double radius, double height) :
        radius_(radius),
        height_(height)
    {
    }

    Point3D getPoint(double u, double v) const override
    {
        return Point3D(radius_ * std::cos(u), radius_ * std::sin(u), v);
    }

    std::array<double, 3> getNormal(double u, double /*v*/) const override
    {
        return {std::cos(u), std::sin(u), 0.0};
    }

    Common::BoundingBox2D getParameterBounds() const override
    {
        return Common::BoundingBox2D(0.0, 2.0 * M_PI, 0.0, height_);
    }

    double getGap(const Point3D& point) const override
    {
        const double radialDistance = std::sqrt(point.x() * point.x() + point.y() * point.y());
        return std::abs(radialDistance - radius_);
    }

    Point2D projectPoint(const Point3D& point) const override
    {
        return Point2D(std::atan2(point.y(), point.x()), point.z());
    }

    std::optional<Point2D> projectPointToUnderlyingSurface(const Point3D& point) const override
    {
        const double radialDistance = std::sqrt(point.x() * point.x() + point.y() * point.y());
        if (radialDistance < 1e-12)
            return std::nullopt;
        return Point2D(std::atan2(point.y(), point.x()), point.z());
    }

    std::optional<Point2D> projectPointToUnderlyingSurface(
        const Point3D& point, const Point2D& /*seedUV*/) const override
    {
        return projectPointToUnderlyingSurface(point);
    }

    std::string getId() const override { return "cylinder"; }

private:
    double radius_;
    double height_;
};

} // namespace

// ============================================================================
// Fixture
// ============================================================================

class SurfaceProjectorTest : public ::testing::Test
{
protected:
    static constexpr double RADIUS = 1.0;
    static constexpr double HEIGHT = 2.0;

    MockCylinderSurface cylinder{RADIUS, HEIGHT};
    SurfaceProjector projector;
};

// ============================================================================
// signedDistance
// ============================================================================

TEST_F(SurfaceProjectorTest, SignedDistancePositiveOutside)
{
    // Point at r = 1.5, well outside the cylinder
    const Point3D point(1.5, 0.0, 1.0);
    EXPECT_NEAR(projector.signedDistance(point, cylinder), 0.5, 1e-10);
}

TEST_F(SurfaceProjectorTest, SignedDistanceNegativeInside)
{
    // Point at r = 0.5, inside the cylinder
    const Point3D point(0.5, 0.0, 1.0);
    EXPECT_NEAR(projector.signedDistance(point, cylinder), -0.5, 1e-10);
}

TEST_F(SurfaceProjectorTest, SignedDistanceZeroOnSurface)
{
    // Point on the cylinder surface
    const Point3D point(RADIUS, 0.0, 1.0);
    EXPECT_NEAR(projector.signedDistance(point, cylinder), 0.0, 1e-10);
}

TEST_F(SurfaceProjectorTest, SignedDistanceConsistentNearSeam)
{
    // Verify sign is consistent for points near the seam (angle ≈ ±π)
    // These use atan2 values near ±π, where OCC parameterization is most likely
    // to flip the normal direction — the key assumption being tested.
    const double angleNearSeam = M_PI - 1e-6;
    const Point3D outsideNearSeam(
        1.5 * std::cos(angleNearSeam), 1.5 * std::sin(angleNearSeam), 1.0);
    const Point3D insideNearSeam(
        0.5 * std::cos(angleNearSeam), 0.5 * std::sin(angleNearSeam), 1.0);

    EXPECT_GT(projector.signedDistance(outsideNearSeam, cylinder), 0.0);
    EXPECT_LT(projector.signedDistance(insideNearSeam, cylinder), 0.0);
}

// ============================================================================
// crossesSurface
// ============================================================================

TEST_F(SurfaceProjectorTest, CrossesSurface_Straddling)
{
    const Point3D inside(0.5, 0.0, 1.0);
    const Point3D outside(1.5, 0.0, 1.0);
    EXPECT_TRUE(projector.crossesSurface(inside, outside, cylinder));
}

TEST_F(SurfaceProjectorTest, CrossesSurface_BothOutside)
{
    const Point3D outside1(1.5, 0.0, 1.0);
    const Point3D outside2(0.0, 1.5, 1.0);
    EXPECT_FALSE(projector.crossesSurface(outside1, outside2, cylinder));
}

TEST_F(SurfaceProjectorTest, CrossesSurface_BothInside)
{
    const Point3D inside1(0.5, 0.0, 1.0);
    const Point3D inside2(0.0, 0.5, 1.0);
    EXPECT_FALSE(projector.crossesSurface(inside1, inside2, cylinder));
}

TEST_F(SurfaceProjectorTest, CrossesSurface_NearTangent)
{
    // Both endpoints sit at tiny signed distances with opposite signs — a segment
    // nearly parallel to the surface. The near-tangent guard falls back to the
    // midpoint, which is on the surface (signedDistance ≈ 0), so this should
    // NOT be classified as crossing (midpoint distance < tangentGuard as well).
    //
    // Construct using the actual NEAR_TANGENT_RELATIVE_TOLERANCE threshold:
    // tangentGuard = 1e-10 * diameter.  diameter ≈ 2*R = 2.0 for the unit cylinder.
    // We put both endpoints just inside ±tangentGuard/2 of the surface.
    const double diameter = 2.0 * RADIUS;
    constexpr double NEAR_TANGENT_RELATIVE_TOLERANCE = 1e-10;
    const double tangentGuard = NEAR_TANGENT_RELATIVE_TOLERANCE * diameter;

    const double epsilon = tangentGuard * 0.4;
    const Point3D almostOnSurface1(RADIUS + epsilon, 0.0, 1.0);
    const Point3D almostOnSurface2(RADIUS - epsilon, 0.0, 1.0);

    // Both endpoints are within tangentGuard of the surface; midpoint is on the
    // surface (signed distance = 0), so the result is false (no crossing).
    EXPECT_FALSE(projector.crossesSurface(almostOnSurface1, almostOnSurface2, cylinder));
}

// ============================================================================
// projectToSurface
// ============================================================================

TEST_F(SurfaceProjectorTest, ProjectToSurface_RoundTrip)
{
    // A point known to lie on the cylinder surface should project back to itself.
    const double angle = 0.7;
    const Point3D onSurface(RADIUS * std::cos(angle), RADIUS * std::sin(angle), 1.2);

    const auto result = projector.projectToSurface(onSurface, cylinder);

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->x(), onSurface.x(), 1e-10);
    EXPECT_NEAR(result->y(), onSurface.y(), 1e-10);
    EXPECT_NEAR(result->z(), onSurface.z(), 1e-10);
}

TEST_F(SurfaceProjectorTest, ProjectToSurface_FarPoint_ProjectsCorrectly)
{
    // A point far from the surface (gap = 5.0) must still project onto the cylinder.
    // The gap guard was removed (OPE-150); projectPointToUnderlyingSurface handles
    // arbitrary distances.
    const Point3D farPoint(6.0, 0.0, 1.0);
    const auto result = projector.projectToSurface(farPoint, cylinder);

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->x(), RADIUS, 1e-10);
    EXPECT_NEAR(result->y(), 0.0, 1e-10);
    EXPECT_NEAR(result->z(), 1.0, 1e-10);
}
