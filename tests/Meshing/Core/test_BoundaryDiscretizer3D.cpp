#include "Common/BoundingBox2D.h"
#include "Common/Types.h"
#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ICorner3D.h"
#include "Geometry/3D/Base/IEdge3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Core/3D/General/BoundaryDiscretizer3D.h"
#include "Meshing/Core/3D/Surface/TwinTableGenerator.h"
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

TEST(BoundaryDiscretizer3D, SharedEdge_OneSegment_IsTwinned)
{
    TriangleStripFixture fix;

    auto twinTable = TwinTableGenerator::generate(*fix.topology);
    // 1 segment per edge → E23 edge points = [idx_C2, idx_C3]
    // 1 surface sample direction → no surface interior points
    Geometry3D::DiscretizationSettings3D settings(1, 1);
    BoundaryDiscretizer3D disc(*fix.geometry, *fix.topology, settings, twinTable);
    disc.discretize();

    const auto& result = disc.getDiscretizationResult();
    const auto& tm     = disc.getTwinManager();

    size_t idxC2 = result.cornerIdToPointIndexMap.at("C2");
    size_t idxC3 = result.cornerIdToPointIndexMap.at("C3");

    // E23: S1=Same → (C2,C3), S2=Reversed → (C3,C2)
    auto twin = tm.getTwin(idxC2, idxC3);
    ASSERT_TRUE(twin.has_value());
    EXPECT_EQ(twin->first,  idxC3);
    EXPECT_EQ(twin->second, idxC2);

    // Symmetric lookup
    auto twinRev = tm.getTwin(idxC3, idxC2);
    ASSERT_TRUE(twinRev.has_value());
    EXPECT_EQ(twinRev->first,  idxC2);
    EXPECT_EQ(twinRev->second, idxC3);
}

TEST(BoundaryDiscretizer3D, SharedEdge_TwoSegments_BothPairsTwinned)
{
    TriangleStripFixture fix;

    auto twinTable = TwinTableGenerator::generate(*fix.topology);
    // 2 segments per edge → E23 edge points = [idx_C2, idx_mid, idx_C3]
    Geometry3D::DiscretizationSettings3D settings(2, 1);
    BoundaryDiscretizer3D disc(*fix.geometry, *fix.topology, settings, twinTable);
    disc.discretize();

    const auto& result = disc.getDiscretizationResult();
    const auto& tm     = disc.getTwinManager();

    const auto& e23pts = result.edgeIdToPointIndicesMap.at("E23");
    ASSERT_EQ(e23pts.size(), 3u); // [C2, mid, C3]

    size_t idxC2  = e23pts[0];
    size_t idxMid = e23pts[1];
    size_t idxC3  = e23pts[2];

    // N=2
    // i=0: Same (C2,mid) ↔ Reversed (C3,mid)
    auto t0 = tm.getTwin(idxC2, idxMid);
    ASSERT_TRUE(t0.has_value());
    EXPECT_EQ(t0->first,  idxC3);
    EXPECT_EQ(t0->second, idxMid);

    // i=1: Same (mid,C3) ↔ Reversed (mid,C2)
    auto t1 = tm.getTwin(idxMid, idxC3);
    ASSERT_TRUE(t1.has_value());
    EXPECT_EQ(t1->first,  idxMid);
    EXPECT_EQ(t1->second, idxC2);
}

TEST(BoundaryDiscretizer3D, BoundaryEdges_HaveNoTwins)
{
    TriangleStripFixture fix;

    auto twinTable = TwinTableGenerator::generate(*fix.topology);
    Geometry3D::DiscretizationSettings3D settings(1, 1);
    BoundaryDiscretizer3D disc(*fix.geometry, *fix.topology, settings, twinTable);
    disc.discretize();

    const auto& result = disc.getDiscretizationResult();
    const auto& tm     = disc.getTwinManager();

    for (const char* edgeId : {"E12", "E13", "E24", "E34"})
    {
        const auto& pts = result.edgeIdToPointIndicesMap.at(edgeId);
        EXPECT_FALSE(tm.hasTwin(pts.front(), pts.back())) << "Unexpected twin for " << edgeId;
        EXPECT_FALSE(tm.hasTwin(pts.back(), pts.front())) << "Unexpected twin for " << edgeId;
    }
}

TEST(BoundaryDiscretizer3D, NoTwinTable_TwinManagerIsEmpty)
{
    TriangleStripFixture fix;

    // No EdgeTwinTable supplied — default {} is empty
    Geometry3D::DiscretizationSettings3D settings(1, 1);
    BoundaryDiscretizer3D disc(*fix.geometry, *fix.topology, settings);
    disc.discretize();

    const auto& result = disc.getDiscretizationResult();
    const auto& tm     = disc.getTwinManager();

    size_t idxC2 = result.cornerIdToPointIndexMap.at("C2");
    size_t idxC3 = result.cornerIdToPointIndexMap.at("C3");

    EXPECT_FALSE(tm.hasTwin(idxC2, idxC3));
}
