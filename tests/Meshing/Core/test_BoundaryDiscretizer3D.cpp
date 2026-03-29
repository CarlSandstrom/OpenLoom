#include "Common/BoundingBox2D.h"
#include "Common/Types.h"
#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ICorner3D.h"
#include "Geometry/3D/Base/IEdge3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Core/3D/General/BoundaryDiscretizer3D.h"
#include "Topology/Corner3D.h"
#include "Topology/Edge3D.h"
#include "Topology/Surface3D.h"
#include "Topology/Topology3D.h"
#include <gtest/gtest.h>
#include <memory>
#include <unordered_map>

using namespace Meshing;

// ============================================================================
// Mock geometry classes
// ============================================================================

namespace
{

class MockCorner3D : public Geometry3D::ICorner3D
{
public:
    MockCorner3D(const std::string& id, const Point3D& point) :
        id_(id), point_(point) {}

    Point3D getPoint() const override { return point_; }
    std::string getId() const override { return id_; }

private:
    std::string id_;
    Point3D point_;
};

class MockEdge3D : public Geometry3D::IEdge3D
{
public:
    MockEdge3D(const std::string& id, const Point3D& start, const Point3D& end) :
        id_(id), start_(start), end_(end) {}

    Point3D getPoint(double t) const override { return start_ + t * (end_ - start_); }
    std::array<double, 3> getTangent(double /*t*/) const override
    {
        Point3D dir = (end_ - start_).normalized();
        return {dir.x(), dir.y(), dir.z()};
    }
    Point3D getStartPoint() const override { return start_; }
    Point3D getEndPoint() const override { return end_; }
    std::pair<double, double> getParameterBounds() const override { return {0.0, 1.0}; }
    double getLength() const override { return (end_ - start_).norm(); }
    double getParameterAtArcLengthFraction(double tStart, double tEnd, double fraction) const override { return tStart + fraction * (tEnd - tStart); }
    double getCurvature(double /*t*/) const override { return 0.0; }
    std::string getId() const override { return id_; }

private:
    std::string id_;
    Point3D start_;
    Point3D end_;
};

class MockPlanarSurface : public Geometry3D::ISurface3D
{
public:
    explicit MockPlanarSurface(const std::string& id) : id_(id) {}

    std::array<double, 3> getNormal(double /*u*/, double /*v*/) const override
    {
        return {0.0, 0.0, 1.0};
    }
    Point3D getPoint(double u, double v) const override { return Point3D(u, v, 0.0); }
    Common::BoundingBox2D getParameterBounds() const override
    {
        return Common::BoundingBox2D(0.0, 1.0, 0.0, 1.0);
    }
    double getGap(const Point3D& point) const override { return std::abs(point.z()); }
    Point2D projectPoint(const Point3D& point) const override
    {
        return Point2D(point.x(), point.y());
    }

    std::optional<Point2D> projectPointToUnderlyingSurface(const Point3D& point) const override
    {
        return Point2D(point.x(), point.y());
    }

    std::optional<Point2D> projectPointToUnderlyingSurface(
        const Point3D& point, const Point2D& /*seedUV*/) const override
    {
        return Point2D(point.x(), point.y());
    }

    std::string getId() const override { return id_; }

private:
    std::string id_;
};

// ============================================================================
// Triangle strip fixture
//
//   C1 ──E12── C2 ──E24── C4
//    \          |          /
//    E13       E23        E34
//      \        |          /
//       ────── C3 ─────────
//
//  S1 = triangle(C1, C2, C3): boundary E12, E13; shared E23 (Same)
//  S2 = triangle(C2, C4, C3): boundary E24, E34; shared E23 (Reversed)
// ============================================================================

struct TriangleStripFixture
{
    std::unique_ptr<Geometry3D::GeometryCollection3D> geometry;
    std::unique_ptr<Topology3D::Topology3D> topology;

    TriangleStripFixture()
    {
        Point3D ptC1(0.0, 0.0, 0.0);
        Point3D ptC2(1.0, 0.0, 0.0);
        Point3D ptC3(0.5, 1.0, 0.0);
        Point3D ptC4(1.5, 1.0, 0.0);

        std::unordered_map<std::string, std::unique_ptr<Geometry3D::ICorner3D>> corners;
        corners["C1"] = std::make_unique<MockCorner3D>("C1", ptC1);
        corners["C2"] = std::make_unique<MockCorner3D>("C2", ptC2);
        corners["C3"] = std::make_unique<MockCorner3D>("C3", ptC3);
        corners["C4"] = std::make_unique<MockCorner3D>("C4", ptC4);

        std::unordered_map<std::string, std::unique_ptr<Geometry3D::IEdge3D>> edges;
        edges["E12"] = std::make_unique<MockEdge3D>("E12", ptC1, ptC2);
        edges["E13"] = std::make_unique<MockEdge3D>("E13", ptC1, ptC3);
        edges["E23"] = std::make_unique<MockEdge3D>("E23", ptC2, ptC3);
        edges["E24"] = std::make_unique<MockEdge3D>("E24", ptC2, ptC4);
        edges["E34"] = std::make_unique<MockEdge3D>("E34", ptC3, ptC4);

        std::unordered_map<std::string, std::unique_ptr<Geometry3D::ISurface3D>> surfaces;
        surfaces["S1"] = std::make_unique<MockPlanarSurface>("S1");
        surfaces["S2"] = std::make_unique<MockPlanarSurface>("S2");

        geometry = std::make_unique<Geometry3D::GeometryCollection3D>(
            std::move(surfaces), std::move(edges), std::move(corners));

        std::unordered_map<std::string, Topology3D::Corner3D> topoCorners;
        topoCorners.emplace("C1", Topology3D::Corner3D("C1", {"E12", "E13"}, {"S1"}));
        topoCorners.emplace("C2", Topology3D::Corner3D("C2", {"E12", "E23", "E24"}, {"S1", "S2"}));
        topoCorners.emplace("C3", Topology3D::Corner3D("C3", {"E13", "E23", "E34"}, {"S1", "S2"}));
        topoCorners.emplace("C4", Topology3D::Corner3D("C4", {"E24", "E34"}, {"S2"}));

        std::unordered_map<std::string, Topology3D::Edge3D> topoEdges;
        topoEdges.emplace("E12", Topology3D::Edge3D("E12", "C1", "C2", {"S1"}));
        topoEdges.emplace("E13", Topology3D::Edge3D("E13", "C1", "C3", {"S1"}));
        topoEdges.emplace("E23", Topology3D::Edge3D("E23", "C2", "C3", {"S1", "S2"}));
        topoEdges.emplace("E24", Topology3D::Edge3D("E24", "C2", "C4", {"S2"}));
        topoEdges.emplace("E34", Topology3D::Edge3D("E34", "C3", "C4", {"S2"}));

        std::unordered_map<std::string, Topology3D::Surface3D> topoSurfaces;
        topoSurfaces.emplace("S1", Topology3D::Surface3D("S1", {"E12", "E23", "E13"}, {"C1", "C2", "C3"}, {"S2"}));
        topoSurfaces.emplace("S2", Topology3D::Surface3D("S2", {"E23", "E24", "E34"}, {"C2", "C4", "C3"}, {"S1"}));

        topology = std::make_unique<Topology3D::Topology3D>(topoSurfaces, topoEdges, topoCorners);
    }
};

} // namespace

// ============================================================================
// Tests
// ============================================================================

TEST(BoundaryDiscretizer3D, CornerPoints_ArePresent)
{
    TriangleStripFixture fix;

    Geometry3D::DiscretizationSettings3D settings(1, 1);
    BoundaryDiscretizer3D disc(*fix.geometry, *fix.topology, settings);
    disc.discretize();

    const auto& result = disc.getDiscretizationResult();

    EXPECT_TRUE(result.cornerIdToPointIndexMap.contains("C1"));
    EXPECT_TRUE(result.cornerIdToPointIndexMap.contains("C2"));
    EXPECT_TRUE(result.cornerIdToPointIndexMap.contains("C3"));
    EXPECT_TRUE(result.cornerIdToPointIndexMap.contains("C4"));
    EXPECT_EQ(result.cornerIdToPointIndexMap.size(), 4u);
}

TEST(BoundaryDiscretizer3D, EdgePoints_OneSegment_EndpointsOnly)
{
    TriangleStripFixture fix;

    Geometry3D::DiscretizationSettings3D settings(1, 1);
    BoundaryDiscretizer3D disc(*fix.geometry, *fix.topology, settings);
    disc.discretize();

    const auto& result = disc.getDiscretizationResult();

    // 1 segment per edge → edge sequence is just [start, end]
    ASSERT_TRUE(result.edgeIdToPointIndicesMap.contains("E23"));
    const auto& e23pts = result.edgeIdToPointIndicesMap.at("E23");
    ASSERT_EQ(e23pts.size(), 2u);

    size_t idxC2 = result.cornerIdToPointIndexMap.at("C2");
    size_t idxC3 = result.cornerIdToPointIndexMap.at("C3");
    EXPECT_EQ(e23pts[0], idxC2);
    EXPECT_EQ(e23pts[1], idxC3);
}

TEST(BoundaryDiscretizer3D, EdgePoints_TwoSegments_HasMidpoint)
{
    TriangleStripFixture fix;

    Geometry3D::DiscretizationSettings3D settings(2, 1);
    BoundaryDiscretizer3D disc(*fix.geometry, *fix.topology, settings);
    disc.discretize();

    const auto& result = disc.getDiscretizationResult();

    ASSERT_TRUE(result.edgeIdToPointIndicesMap.contains("E23"));
    const auto& e23pts = result.edgeIdToPointIndicesMap.at("E23");
    ASSERT_EQ(e23pts.size(), 3u); // [C2, mid, C3]
}
