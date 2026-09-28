#include "Common/BoundingBox2D.h"
#include "Common/Types.h"
#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ICorner3D.h"
#include "Geometry/3D/Base/IEdge3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Core/3D/General/BoundaryDiscretizer3D.h"
#include "Meshing/Core/3D/General/SizingField3D.h"
#include "Topology/Corner3D.h"
#include "Topology/Edge3D.h"
#include "Topology/SeamCollection.h"
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
    Vector3D getTangent(double /*t*/) const override
    {
        return (end_ - start_).normalized();
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

    Vector3D getNormal(double /*u*/, double /*v*/) const override
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
        Point3D pointC1(0.0, 0.0, 0.0);
        Point3D pointC2(1.0, 0.0, 0.0);
        Point3D pointC3(0.5, 1.0, 0.0);
        Point3D pointC4(1.5, 1.0, 0.0);

        std::unordered_map<std::string, std::unique_ptr<Geometry3D::ICorner3D>> corners;
        corners["C1"] = std::make_unique<MockCorner3D>("C1", pointC1);
        corners["C2"] = std::make_unique<MockCorner3D>("C2", pointC2);
        corners["C3"] = std::make_unique<MockCorner3D>("C3", pointC3);
        corners["C4"] = std::make_unique<MockCorner3D>("C4", pointC4);

        std::unordered_map<std::string, std::unique_ptr<Geometry3D::IEdge3D>> edges;
        edges["E12"] = std::make_unique<MockEdge3D>("E12", pointC1, pointC2);
        edges["E13"] = std::make_unique<MockEdge3D>("E13", pointC1, pointC3);
        edges["E23"] = std::make_unique<MockEdge3D>("E23", pointC2, pointC3);
        edges["E24"] = std::make_unique<MockEdge3D>("E24", pointC2, pointC4);
        edges["E34"] = std::make_unique<MockEdge3D>("E34", pointC3, pointC4);

        std::unordered_map<std::string, std::unique_ptr<Geometry3D::ISurface3D>> surfaces;
        surfaces["S1"] = std::make_unique<MockPlanarSurface>("S1");
        surfaces["S2"] = std::make_unique<MockPlanarSurface>("S2");

        geometry = std::make_unique<Geometry3D::GeometryCollection3D>(
            std::move(surfaces), std::move(edges), std::move(corners));

        std::unordered_map<std::string, Topology3D::Corner3D> topologyCorners;
        topologyCorners.emplace("C1", Topology3D::Corner3D("C1", {"E12", "E13"}, {"S1"}));
        topologyCorners.emplace("C2", Topology3D::Corner3D("C2", {"E12", "E23", "E24"}, {"S1", "S2"}));
        topologyCorners.emplace("C3", Topology3D::Corner3D("C3", {"E13", "E23", "E34"}, {"S1", "S2"}));
        topologyCorners.emplace("C4", Topology3D::Corner3D("C4", {"E24", "E34"}, {"S2"}));

        std::unordered_map<std::string, Topology3D::Edge3D> topologyEdges;
        topologyEdges.emplace("E12", Topology3D::Edge3D("E12", "C1", "C2", {"S1"}));
        topologyEdges.emplace("E13", Topology3D::Edge3D("E13", "C1", "C3", {"S1"}));
        topologyEdges.emplace("E23", Topology3D::Edge3D("E23", "C2", "C3", {"S1", "S2"}));
        topologyEdges.emplace("E24", Topology3D::Edge3D("E24", "C2", "C4", {"S2"}));
        topologyEdges.emplace("E34", Topology3D::Edge3D("E34", "C3", "C4", {"S2"}));

        std::unordered_map<std::string, Topology3D::Surface3D> topologySurfaces;
        topologySurfaces.emplace("S1", Topology3D::Surface3D("S1", {"E12", "E23", "E13"}, {"C1", "C2", "C3"}, {"S2"}));
        topologySurfaces.emplace("S2", Topology3D::Surface3D("S2", {"E23", "E24", "E34"}, {"C2", "C4", "C3"}, {"S1"}));

        topology = std::make_unique<Topology3D::Topology3D>(topologySurfaces, topologyEdges, topologyCorners);
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
    const auto discretizationResult =
        BoundaryDiscretizer3D::discretize(*fix.geometry, *fix.topology, settings);
    const auto& result = *discretizationResult;

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
    const auto discretizationResult =
        BoundaryDiscretizer3D::discretize(*fix.geometry, *fix.topology, settings);
    const auto& result = *discretizationResult;

    // 1 segment per edge → edge sequence is just [start, end]
    ASSERT_TRUE(result.edgeIdToPointIndicesMap.contains("E23"));
    const auto& e23pts = result.edgeIdToPointIndicesMap.at("E23");
    ASSERT_EQ(e23pts.size(), 2u);

    size_t indexC2 = result.cornerIdToPointIndexMap.at("C2");
    size_t indexC3 = result.cornerIdToPointIndexMap.at("C3");
    EXPECT_EQ(e23pts[0], indexC2);
    EXPECT_EQ(e23pts[1], indexC3);
}

TEST(BoundaryDiscretizer3D, EdgePoints_TwoSegments_HasMidpoint)
{
    TriangleStripFixture fix;

    Geometry3D::DiscretizationSettings3D settings(2, 1);
    const auto discretizationResult =
        BoundaryDiscretizer3D::discretize(*fix.geometry, *fix.topology, settings);
    const auto& result = *discretizationResult;

    ASSERT_TRUE(result.edgeIdToPointIndicesMap.contains("E23"));
    const auto& e23pts = result.edgeIdToPointIndicesMap.at("E23");
    ASSERT_EQ(e23pts.size(), 3u); // [C2, mid, C3]
}

TEST(BoundaryDiscretizer3D, EdgePoints_InteriorPoint_HasCorrectEdgeParameter)
{
    TriangleStripFixture fixture;

    Geometry3D::DiscretizationSettings3D settings(2, 0);
    const auto discretizationResult =
        BoundaryDiscretizer3D::discretize(*fixture.geometry, *fixture.topology, settings);
    const auto& result = *discretizationResult;

    ASSERT_TRUE(result.edgeIdToPointIndicesMap.contains("E12"));
    const auto& edgePointIndices = result.edgeIdToPointIndicesMap.at("E12");
    ASSERT_EQ(edgePointIndices.size(), 3u); // start, interior, end

    size_t interiorIndex = edgePointIndices[1];
    ASSERT_EQ(result.edgeParameters[interiorIndex].size(), 1u);
    EXPECT_DOUBLE_EQ(result.edgeParameters[interiorIndex][0], 0.5);
}

TEST(BoundaryDiscretizer3D, EdgePoints_TwoInteriorPoints_HaveCorrectEdgeParameters)
{
    TriangleStripFixture fixture;

    Geometry3D::DiscretizationSettings3D settings(3, 0);
    const auto discretizationResult =
        BoundaryDiscretizer3D::discretize(*fixture.geometry, *fixture.topology, settings);
    const auto& result = *discretizationResult;

    ASSERT_TRUE(result.edgeIdToPointIndicesMap.contains("E12"));
    const auto& edgePointIndices = result.edgeIdToPointIndicesMap.at("E12");
    ASSERT_EQ(edgePointIndices.size(), 4u); // start, interior1, interior2, end

    ASSERT_EQ(result.edgeParameters[edgePointIndices[1]].size(), 1u);
    ASSERT_EQ(result.edgeParameters[edgePointIndices[2]].size(), 1u);
    EXPECT_DOUBLE_EQ(result.edgeParameters[edgePointIndices[1]][0], 1.0 / 3.0);
    EXPECT_DOUBLE_EQ(result.edgeParameters[edgePointIndices[2]][0], 2.0 / 3.0);
}

// The length-driven walk emits a point whenever the accumulated arc length
// reaches the bound, and E12's length is exactly four times the bound here --
// so the last emission lands exactly on the end vertex. That vertex is already
// a point of the edge (it is the corner), so emitting there duplicates it, and
// the regular triangulation can only keep one of two coincident weighted
// nodes; the other ends up in no triangle at all. Measured on the torus with a
// sizing field before this was guarded: one orphaned corner node, four
// AllEdgeNodesCovered failures (OPE-207).
TEST(BoundaryDiscretizer3D, LengthDrivenWalk_DoesNotDuplicateTheEndVertex)
{
    TriangleStripFixture fix;

    // Size far larger than the edge, so h(x) never binds and the bound comes
    // from the count alone: E12 has length 1, so the bound is exactly 0.25.
    const SizingField3D sizingField({{Point3D(0.0, 0.0, 0.0), 100.0}}, 0.3);

    Geometry3D::DiscretizationSettings3D settings(4, 1);
    const auto discretizationResult =
        BoundaryDiscretizer3D::discretize(*fix.geometry, *fix.topology, settings, &sizingField);
    const auto& result = *discretizationResult;

    ASSERT_TRUE(result.edgeIdToPointIndicesMap.contains("E12"));
    const auto& edgePointIndices = result.edgeIdToPointIndicesMap.at("E12");

    const size_t endCornerIndex = result.cornerIdToPointIndexMap.at("C2");
    const Point3D endVertex = result.points[endCornerIndex];

    // The end vertex appears exactly once, as the chain's last entry.
    EXPECT_EQ(edgePointIndices.back(), endCornerIndex);
    for (size_t i = 0; i + 1 < edgePointIndices.size(); ++i)
    {
        EXPECT_GT((result.points[edgePointIndices[i]] - endVertex).norm(), 1e-9)
            << "point " << i << " of E12 coincides with the end vertex";
    }

    // Four segments: the two corners plus three interior points.
    EXPECT_EQ(edgePointIndices.size(), 5u);
}

TEST(BoundaryDiscretizer3D, SeamTwinEdge_SequenceIsReverseOfOriginalEdge)
{
    Point3D pointCA(0.0, 0.0, 0.0);
    Point3D pointCB(1.0, 0.0, 0.0);

    std::unordered_map<std::string, std::unique_ptr<Geometry3D::ICorner3D>> corners;
    corners["CA"] = std::make_unique<MockCorner3D>("CA", pointCA);
    corners["CB"] = std::make_unique<MockCorner3D>("CB", pointCB);

    std::unordered_map<std::string, std::unique_ptr<Geometry3D::IEdge3D>> edges;
    edges["seam"] = std::make_unique<MockEdge3D>("seam", pointCA, pointCB);
    // seam_twin has no 3D geometry — the discretizer derives its sequence by reversing "seam"

    std::unordered_map<std::string, std::unique_ptr<Geometry3D::ISurface3D>> surfaces;

    auto geometry = std::make_unique<Geometry3D::GeometryCollection3D>(
        std::move(surfaces), std::move(edges), std::move(corners));

    std::unordered_map<std::string, Topology3D::Corner3D> topologyCorners;
    topologyCorners.emplace("CA", Topology3D::Corner3D("CA", {"seam", "seam_twin"}, {}));
    topologyCorners.emplace("CB", Topology3D::Corner3D("CB", {"seam", "seam_twin"}, {}));

    std::unordered_map<std::string, Topology3D::Edge3D> topologyEdges;
    topologyEdges.emplace("seam", Topology3D::Edge3D("seam", "CA", "CB", {}));
    topologyEdges.emplace("seam_twin", Topology3D::Edge3D("seam_twin", "CB", "CA", {}));

    std::unordered_map<std::string, Topology3D::Surface3D> topologySurfaces;

    Topology3D::SeamCollection seams;
    seams.addPair("seam", "seam_twin");

    auto topology = std::make_unique<Topology3D::Topology3D>(
        topologySurfaces, topologyEdges, topologyCorners, std::move(seams));

    Geometry3D::DiscretizationSettings3D settings(2, 0);
    const auto discretizationResult =
        BoundaryDiscretizer3D::discretize(*geometry, *topology, settings);
    const auto& result = *discretizationResult;

    ASSERT_TRUE(result.edgeIdToPointIndicesMap.contains("seam"));
    ASSERT_TRUE(result.edgeIdToPointIndicesMap.contains("seam_twin"));

    const auto& originalSequence = result.edgeIdToPointIndicesMap.at("seam");
    const auto& twinSequence = result.edgeIdToPointIndicesMap.at("seam_twin");

    ASSERT_EQ(originalSequence.size(), twinSequence.size());
    for (size_t i = 0; i < originalSequence.size(); ++i)
        EXPECT_EQ(originalSequence[i], twinSequence[originalSequence.size() - 1 - i]);
}

TEST(BoundaryDiscretizer3D, SurfaceInterior_NonzeroSamples_SurfaceMapIsPopulated)
{
    TriangleStripFixture fixture;

    // 2 samples per surface direction → 1×1 = 1 interior point per surface
    Geometry3D::DiscretizationSettings3D settings(1, 2);
    const auto discretizationResult =
        BoundaryDiscretizer3D::discretize(*fixture.geometry, *fixture.topology, settings);
    const auto& result = *discretizationResult;

    ASSERT_TRUE(result.surfaceIdToPointIndicesMap.contains("S1"));
    ASSERT_TRUE(result.surfaceIdToPointIndicesMap.contains("S2"));
    EXPECT_EQ(result.surfaceIdToPointIndicesMap.at("S1").size(), 1u);
    EXPECT_EQ(result.surfaceIdToPointIndicesMap.at("S2").size(), 1u);

    size_t s1InteriorIndex = result.surfaceIdToPointIndicesMap.at("S1")[0];
    ASSERT_EQ(result.geometryIds[s1InteriorIndex].size(), 1u);
    EXPECT_EQ(result.geometryIds[s1InteriorIndex][0], "S1");

    size_t s2InteriorIndex = result.surfaceIdToPointIndicesMap.at("S2")[0];
    ASSERT_EQ(result.geometryIds[s2InteriorIndex].size(), 1u);
    EXPECT_EQ(result.geometryIds[s2InteriorIndex][0], "S2");
}
