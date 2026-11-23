#include "Meshing/Core/3D/General/SurfaceProjector.h"

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

    std::optional<Vector3D> getNormal(double u, double /*v*/) const override
    {
        return Vector3D(std::cos(u), std::sin(u), 0.0);
    }

    Geometry3D::SurfaceDerivatives getFirstDerivatives(double u, double v) const override
    {
        return {getPoint(u, v), {-radius_ * std::sin(u), radius_ * std::cos(u), 0.0}, {0.0, 0.0, 1.0}};
    }

    Common::BoundingBox2D getParameterBounds() const override
    {
        return Common::BoundingBox2D(0.0, 2.0 * M_PI, 0.0, height_);
    }

    std::optional<double> getGap(const Point3D& point) const override
    {
        const double radialDistance = std::sqrt(point.x() * point.x() + point.y() * point.y());
        return std::abs(radialDistance - radius_);
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
};

// ============================================================================
// signedDistance
// ============================================================================

TEST_F(SurfaceProjectorTest, SignedDistancePositiveOutside)
{
    // Point at r = 1.5, well outside the cylinder
    const Point3D point(1.5, 0.0, 1.0);
    EXPECT_NEAR(SurfaceProjector::signedDistance(point, cylinder).value(), 0.5, 1e-10);
}

TEST_F(SurfaceProjectorTest, SignedDistanceNegativeInside)
{
    // Point at r = 0.5, inside the cylinder
    const Point3D point(0.5, 0.0, 1.0);
    EXPECT_NEAR(SurfaceProjector::signedDistance(point, cylinder).value(), -0.5, 1e-10);
}

TEST_F(SurfaceProjectorTest, SignedDistanceZeroOnSurface)
{
    // Point on the cylinder surface
    const Point3D point(RADIUS, 0.0, 1.0);
    EXPECT_NEAR(SurfaceProjector::signedDistance(point, cylinder).value(), 0.0, 1e-10);
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

    EXPECT_GT(SurfaceProjector::signedDistance(outsideNearSeam, cylinder).value(), 0.0);
    EXPECT_LT(SurfaceProjector::signedDistance(insideNearSeam, cylinder).value(), 0.0);
}

// ============================================================================
// projectToSurface
// ============================================================================

TEST_F(SurfaceProjectorTest, ProjectToSurface_RoundTrip)
{
    // A point known to lie on the cylinder surface should project back to itself.
    const double angle = 0.7;
    const Point3D onSurface(RADIUS * std::cos(angle), RADIUS * std::sin(angle), 1.2);

    const auto result = SurfaceProjector::projectToSurface(onSurface, cylinder);

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
    const auto result = SurfaceProjector::projectToSurface(farPoint, cylinder);

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->x(), RADIUS, 1e-10);
    EXPECT_NEAR(result->y(), 0.0, 1e-10);
    EXPECT_NEAR(result->z(), 1.0, 1e-10);
}

// ============================================================================
// findSurfaceCrossing
// ============================================================================

TEST_F(SurfaceProjectorTest, FindSurfaceCrossing_ObliqueSegment_LandsOnSurfaceAndSegment)
{
    // An oblique segment through the curved wall, seeded a few cells' worth of
    // parameter away from the answer, as a coarse tessellation would seed it.
    const Point3D segmentStart(0.6, 0.3, 0.8);
    const Point3D segmentEnd(1.4, 0.5, 1.1);
    const Point2D seedUV(0.5, 0.8);

    const auto crossing = SurfaceProjector::findSurfaceCrossing(segmentStart, segmentEnd, cylinder, seedUV);

    ASSERT_TRUE(crossing.has_value());
    EXPECT_NEAR(std::hypot(crossing->x(), crossing->y()), RADIUS, 1e-12);
    const Vector3D direction = segmentEnd - segmentStart;
    EXPECT_NEAR(direction.cross(*crossing - segmentStart).norm(), 0.0, 1e-12);
}

TEST_F(SurfaceProjectorTest, FindSurfaceCrossing_CrossingOffSegment_ReturnsNullopt)
{
    // The line through the segment crosses the wall at x = 1, past its end.
    const Point3D segmentStart(0.2, 0.0, 1.0);
    const Point3D segmentEnd(0.8, 0.0, 1.0);

    EXPECT_FALSE(SurfaceProjector::findSurfaceCrossing(segmentStart, segmentEnd, cylinder, Point2D(0.0, 1.0)));
}
