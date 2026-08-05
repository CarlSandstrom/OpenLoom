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

// A height field with a narrow spike at u=0, much narrower than a coarse
// tessellation's grid spacing -- mirrors the cylinder's curved lateral
// surface, where a fixed low-resolution tessellation misses fine mesh-scale
// features once refinement shrinks elements below the tessellation's own
// cell size (see RCDTMesherCylinderTest.AllEdgeNodesCovered's iteration-cap
// regression this fixes).
class MockSpikeSurface : public Geometry3D::ISurface3D
{
public:
    // spikeCenter deliberately off both the domain's corners and its exact
    // midpoint -- the spike's own peak has zero slope by symmetry, so a
    // flatness probe that happened to land exactly there would see the same
    // normal as the flat far-field and misjudge the surface as planar.
    MockSpikeSurface(double halfRange, double amplitude, double spikeWidth, double spikeCenter) :
        halfRange_(halfRange),
        amplitude_(amplitude),
        spikeWidth_(spikeWidth),
        spikeCenter_(spikeCenter)
    {
    }

    Point3D getPoint(double u, double v) const override
    {
        const double normalizedU = (u - spikeCenter_) / spikeWidth_;
        return Point3D(u, v, amplitude_ * std::exp(-normalizedU * normalizedU));
    }

    // Analytic normal of the height field z = f(u): tangents are
    // dS/du = (1, 0, f'(u)) and dS/dv = (0, 1, 0), so their cross product is
    // (-f'(u), 0, 1), normalized.
    std::array<double, 3> getNormal(double u, double /*v*/) const override
    {
        const double offset = u - spikeCenter_;
        const double normalizedU = offset / spikeWidth_;
        const double height = amplitude_ * std::exp(-normalizedU * normalizedU);
        const double derivative = height * (-2.0 * offset / (spikeWidth_ * spikeWidth_));
        const Point3D normal = Point3D(-derivative, 0.0, 1.0).normalized();
        return {normal.x(), normal.y(), normal.z()};
    }

    Common::BoundingBox2D getParameterBounds() const override
    {
        return Common::BoundingBox2D(-halfRange_, halfRange_, 0.0, 1.0);
    }

    double getGap(const Point3D& /*point*/) const override { return 0.0; }

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

    std::string getId() const override { return "spike"; }

private:
    double halfRange_;
    double amplitude_;
    double spikeWidth_;
    double spikeCenter_;
};

} // namespace

// ============================================================================
// build() + crossesSurface()
// ============================================================================

TEST(SurfaceTessellationTest, CrossesSurface_StraddlingSegment_ReturnsTrue)
{
    const MockPlanarSurface surface(10.0);
    SurfaceTessellation tessellation;
    tessellation.build(surface, 1.0);

    EXPECT_TRUE(tessellation.crossesSurface(Point3D(5.3, 4.7, -1.0), Point3D(5.3, 4.7, 1.0)));
}

TEST(SurfaceTessellationTest, CrossesSurface_BothEndpointsSameSide_ReturnsFalse)
{
    const MockPlanarSurface surface(10.0);
    SurfaceTessellation tessellation;
    tessellation.build(surface, 1.0);

    EXPECT_FALSE(tessellation.crossesSurface(Point3D(5.3, 4.7, 1.0), Point3D(5.3, 4.7, 2.0)));
}

TEST(SurfaceTessellationTest, CrossesSurface_OutsideParameterBounds_ReturnsFalse)
{
    // Crosses z=0, but at (x,y) = (20,20) -- outside the surface's own
    // [0,10]x[0,10] footprint, so no tessellation triangle should cover it.
    const MockPlanarSurface surface(10.0);
    SurfaceTessellation tessellation;
    tessellation.build(surface, 1.0);

    EXPECT_FALSE(tessellation.crossesSurface(Point3D(20.0, 20.0, -1.0), Point3D(20.0, 20.0, 1.0)));
}

TEST(SurfaceTessellationTest, CrossesSurface_ExtremelyLongSegment_StillDetectsCrossing)
{
    // Mirrors the dual-edge substitution in RestrictedTriangulation, where
    // one endpoint can be a bounding-supertet node hundreds of units away
    // (OPE-169) -- the crossing must still be found.
    const MockPlanarSurface surface(10.0);
    SurfaceTessellation tessellation;
    tessellation.build(surface, 1.0);

    EXPECT_TRUE(tessellation.crossesSurface(Point3D(5.3, 4.7, -1.0), Point3D(5.3, 4.7, 1000.0)));
}

TEST(SurfaceTessellationTest, Build_ZeroTargetCellSize_ProducesNoTriangles)
{
    const MockPlanarSurface surface(10.0);
    SurfaceTessellation tessellation;
    tessellation.build(surface, 0.0);

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
    tessellation.build(surface, 0.5);

    EXPECT_FALSE(tessellation.crossesSurface(Point3D(5.3, 4.7, -1.0), Point3D(5.3, 4.7, 1.0)));
}

TEST(SurfaceTessellationTest, CrossesSurface_AwayFromHole_ReturnsTrue)
{
    const MockPlanarSurfaceWithHole surface(10.0, /*holeRadius=*/2.0);
    SurfaceTessellation tessellation;
    tessellation.build(surface, 0.5);

    EXPECT_TRUE(tessellation.crossesSurface(Point3D(1.3, 0.7, -1.0), Point3D(1.3, 0.7, 1.0)));
}

// ============================================================================
// Resolution scaling for curved surfaces
// ============================================================================

namespace
{
// Off both the domain's corners (+-5) and its exact midpoint (0) -- see
// MockSpikeSurface's constructor doc.
constexpr double SPIKE_CENTER = 1.7;
} // namespace

TEST(SurfaceTessellationTest, CrossesSurface_CoarseTargetCellSizeMissesNarrowSpike)
{
    // targetCellSize = 5.0 on a 10-unit wide surface gives ~3 samples per
    // direction. The spike (width 0.1) is far narrower than the grid spacing,
    // so no sample column lands near it and the tessellation looks flat.
    const MockSpikeSurface surface(/*halfRange=*/5.0, /*amplitude=*/10.0, /*spikeWidth=*/0.1, SPIKE_CENTER);
    SurfaceTessellation tessellation;
    tessellation.build(surface, 5.0);

    // The true surface reaches z=10 at the spike center; a segment spanning
    // z=[8,12] there crosses the true surface but not this coarse oracle.
    EXPECT_FALSE(tessellation.crossesSurface(Point3D(SPIKE_CENTER, 0.5, 8.0), Point3D(SPIKE_CENTER, 0.5, 12.0)));
}

TEST(SurfaceTessellationTest, CrossesSurface_FineTargetCellSizeFindsNarrowSpike)
{
    // targetCellSize = 0.02 on a 10-unit wide surface gives ~400 samples per
    // direction (the cap). Spacing of 0.025 per column guarantees a column
    // lands within the spike's 0.1-unit width at position SPIKE_CENTER.
    const MockSpikeSurface surface(/*halfRange=*/5.0, /*amplitude=*/10.0, /*spikeWidth=*/0.1, SPIKE_CENTER);
    SurfaceTessellation tessellation;
    tessellation.build(surface, 0.02);

    EXPECT_TRUE(tessellation.crossesSurface(Point3D(SPIKE_CENTER, 0.5, 8.0), Point3D(SPIKE_CENTER, 0.5, 12.0)));
}
