#include "Meshing/Core/3D/RCDT/SurfaceTessellation.h"

#include "Common/BoundingBox2D.h"
#include "Common/Types.h"
#include "Geometry/3D/Base/ISurface3D.h"

#include <cmath>
#include <gtest/gtest.h>
#include <optional>

using namespace Meshing;

// ============================================================================
// Mock surfaces
// ============================================================================

namespace
{

// A flat, fully-trimmed square [0, size] x [0, size] in the z = planeZ plane.
class MockPlanarSurface : public Geometry3D::ISurface3D
{
public:
    explicit MockPlanarSurface(double size, double planeZ = 0.0) :
        size_(size),
        planeZ_(planeZ)
    {
    }

    Point3D getPoint(double u, double v) const override { return Point3D(u, v, planeZ_); }

    std::array<double, 3> getNormal(double /*u*/, double /*v*/) const override { return {0.0, 0.0, 1.0}; }

    Common::BoundingBox2D getParameterBounds() const override
    {
        return Common::BoundingBox2D(0.0, size_, 0.0, size_);
    }

    double getGap(const Point3D& point) const override { return std::abs(point.z() - planeZ_); }

    Point2D projectPoint(const Point3D& point) const override { return Point2D(point.x(), point.y()); }

    std::optional<Point2D> projectPointToUnderlyingSurface(const Point3D& point) const override
    {
        return Point2D(point.x(), point.y());
    }

    std::optional<Point2D> projectPointToUnderlyingSurface(const Point3D& point,
                                                            const Point2D& /*seedUV*/) const override
    {
        return projectPointToUnderlyingSurface(point);
    }

    std::string getId() const override { return "planar"; }

private:
    double size_;
    double planeZ_;
};

// Same flat square, but with a circular hole cut out of its center -- mirrors
// the BoxWithHole scenario that motivated this class (OPE-169).
class MockPlanarSurfaceWithHole : public MockPlanarSurface
{
public:
    MockPlanarSurfaceWithHole(double size, double holeRadius) :
        MockPlanarSurface(size),
        holeCenter_(size / 2.0, size / 2.0, 0.0),
        holeRadius_(holeRadius)
    {
    }

    bool isPointWithinTrimmedBoundary(const Point3D& point) const override
    {
        return (point - holeCenter_).norm() >= holeRadius_;
    }

private:
    Point3D holeCenter_;
    double holeRadius_;
};

} // namespace

// ============================================================================
// build() + crossesSurface()
// ============================================================================

TEST(SurfaceTessellationTest, CrossesSurface_StraddlingSegment_ReturnsTrue)
{
    const MockPlanarSurface surface(10.0);
    SurfaceTessellation tessellation;
    tessellation.build(surface, 10);

    EXPECT_TRUE(tessellation.crossesSurface(Point3D(5.3, 4.7, -1.0), Point3D(5.3, 4.7, 1.0)));
}

TEST(SurfaceTessellationTest, CrossesSurface_BothEndpointsSameSide_ReturnsFalse)
{
    const MockPlanarSurface surface(10.0);
    SurfaceTessellation tessellation;
    tessellation.build(surface, 10);

    EXPECT_FALSE(tessellation.crossesSurface(Point3D(5.3, 4.7, 1.0), Point3D(5.3, 4.7, 2.0)));
}

TEST(SurfaceTessellationTest, CrossesSurface_OutsideParameterBounds_ReturnsFalse)
{
    // Crosses z=0, but at (x,y) = (20,20) -- outside the surface's own
    // [0,10]x[0,10] footprint, so no tessellation triangle should cover it.
    const MockPlanarSurface surface(10.0);
    SurfaceTessellation tessellation;
    tessellation.build(surface, 10);

    EXPECT_FALSE(tessellation.crossesSurface(Point3D(20.0, 20.0, -1.0), Point3D(20.0, 20.0, 1.0)));
}

TEST(SurfaceTessellationTest, CrossesSurface_ExtremelyLongSegment_StillDetectsCrossing)
{
    // Mirrors the dual-edge substitution in RestrictedTriangulation, where
    // one endpoint can be a bounding-supertet node hundreds of units away
    // (OPE-169) -- the crossing must still be found.
    const MockPlanarSurface surface(10.0);
    SurfaceTessellation tessellation;
    tessellation.build(surface, 10);

    EXPECT_TRUE(tessellation.crossesSurface(Point3D(5.3, 4.7, -1.0), Point3D(5.3, 4.7, 1000.0)));
}

TEST(SurfaceTessellationTest, Build_TooFewSamples_ProducesNoTriangles)
{
    const MockPlanarSurface surface(10.0);
    SurfaceTessellation tessellation;
    tessellation.build(surface, 1);

    EXPECT_FALSE(tessellation.crossesSurface(Point3D(5.3, 4.7, -1.0), Point3D(5.3, 4.7, 1.0)));
}

// ============================================================================
// Trimmed boundary (hole) handling -- the bug this class exists to fix
// ============================================================================

TEST(SurfaceTessellationTest, CrossesSurface_ThroughHole_ReturnsFalse)
{
    // A segment crossing z=0 at the exact center of a hole cut into the
    // surface must not be classified as crossing the surface there --
    // there's no real material for it to be crossing.
    const MockPlanarSurfaceWithHole surface(10.0, /*holeRadius=*/2.0);
    SurfaceTessellation tessellation;
    tessellation.build(surface, 20);

    EXPECT_FALSE(tessellation.crossesSurface(Point3D(5.3, 4.7, -1.0), Point3D(5.3, 4.7, 1.0)));
}

TEST(SurfaceTessellationTest, CrossesSurface_AwayFromHole_ReturnsTrue)
{
    const MockPlanarSurfaceWithHole surface(10.0, /*holeRadius=*/2.0);
    SurfaceTessellation tessellation;
    tessellation.build(surface, 20);

    EXPECT_TRUE(tessellation.crossesSurface(Point3D(1.3, 0.7, -1.0), Point3D(1.3, 0.7, 1.0)));
}
