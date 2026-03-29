#include <gtest/gtest.h>

#include "Common/BoundingBox2D.h"
#include "Common/Types.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ICorner3D.h"
#include "Geometry/3D/Base/IEdge3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Core/2D/DiscretizationResult2D.h"
#include "Meshing/Core/3D/General/GeometryStructures3D.h"
#include "Meshing/Core/3D/General/FacetTriangulation.h"
#include "Topology/Corner3D.h"
#include "Topology/Edge3D.h"
#include "Topology/Surface3D.h"
#include "Topology/Topology3D.h"

using namespace Meshing;

// ============================================================================
// Mock Geometry Classes for Testing
// ============================================================================

namespace
{

class MockCorner3D : public Geometry3D::ICorner3D
{
public:
    MockCorner3D(const std::string& id, const Point3D& point) :
        id_(id),
        point_(point)
    {
    }

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
        id_(id),
        start_(start),
        end_(end)
    {
    }

    Point3D getPoint(double t) const override
    {
        return start_ + t * (end_ - start_);
    }

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

// A simple planar surface in the XY plane at z=0
// Parametric: (u, v) -> (u, v, 0)
class MockPlanarSurface : public Geometry3D::ISurface3D
{
public:
    explicit MockPlanarSurface(const std::string& id) :
        id_(id)
    {
    }

    std::array<double, 3> getNormal(double /*u*/, double /*v*/) const override
    {
        return {0.0, 0.0, 1.0};
    }

    Point3D getPoint(double u, double v) const override
    {
        return Point3D(u, v, 0.0);
    }

    Common::BoundingBox2D getParameterBounds() const override
    {
        return Common::BoundingBox2D(0.0, 1.0, 0.0, 1.0);
    }

    double getGap(const Point3D& point) const override
    {
        return std::abs(point.z());
    }

    Point2D projectPoint(const Point3D& point) const override
    {
        // For XY plane surface, projection is just (x, y)
        return Point2D(point.x(), point.y());
    }

    std::string getId() const override { return id_; }

private:
    std::string id_;
};

// Helper to create a unit square geometry and topology.
// Corners: c0=(0,0), c1=(1,0), c2=(1,1), c3=(0,1)
// Edges: e0=c0->c1, e1=c1->c2, e2=c2->c3, e3=c3->c0
struct SimpleSquareFixture
{
    SimpleSquareFixture()
    {
        corners["c0"] = std::make_unique<MockCorner3D>("c0", Point3D(0, 0, 0));
        corners["c1"] = std::make_unique<MockCorner3D>("c1", Point3D(1, 0, 0));
        corners["c2"] = std::make_unique<MockCorner3D>("c2", Point3D(1, 1, 0));
        corners["c3"] = std::make_unique<MockCorner3D>("c3", Point3D(0, 1, 0));

        edges["e0"] = std::make_unique<MockEdge3D>("e0", Point3D(0, 0, 0), Point3D(1, 0, 0));
        edges["e1"] = std::make_unique<MockEdge3D>("e1", Point3D(1, 0, 0), Point3D(1, 1, 0));
        edges["e2"] = std::make_unique<MockEdge3D>("e2", Point3D(1, 1, 0), Point3D(0, 1, 0));
        edges["e3"] = std::make_unique<MockEdge3D>("e3", Point3D(0, 1, 0), Point3D(0, 0, 0));

        surfaces["s0"] = std::make_unique<MockPlanarSurface>("s0");

        topoCorners.emplace("c0", Topology3D::Corner3D("c0", {"e0", "e3"}, {"s0"}));
        topoCorners.emplace("c1", Topology3D::Corner3D("c1", {"e0", "e1"}, {"s0"}));
        topoCorners.emplace("c2", Topology3D::Corner3D("c2", {"e1", "e2"}, {"s0"}));
        topoCorners.emplace("c3", Topology3D::Corner3D("c3", {"e2", "e3"}, {"s0"}));

        topoEdges.emplace("e0", Topology3D::Edge3D("e0", "c0", "c1", {"s0"}));
        topoEdges.emplace("e1", Topology3D::Edge3D("e1", "c1", "c2", {"s0"}));
        topoEdges.emplace("e2", Topology3D::Edge3D("e2", "c2", "c3", {"s0"}));
        topoEdges.emplace("e3", Topology3D::Edge3D("e3", "c3", "c0", {"s0"}));

        topoSurfaces.emplace("s0", Topology3D::Surface3D(
            "s0",
            {"e0", "e1", "e2", "e3"},
            {"c0", "c1", "c2", "c3"}));
    }

    std::unique_ptr<Geometry3D::GeometryCollection3D> createGeometry()
    {
        std::unordered_map<std::string, std::unique_ptr<Geometry3D::ISurface3D>> surfaceMap;
        std::unordered_map<std::string, std::unique_ptr<Geometry3D::IEdge3D>> edgeMap;
        std::unordered_map<std::string, std::unique_ptr<Geometry3D::ICorner3D>> cornerMap;

        for (auto& [id, ptr] : surfaces) surfaceMap[id] = std::move(ptr);
        for (auto& [id, ptr] : edges) edgeMap[id] = std::move(ptr);
        for (auto& [id, ptr] : corners) cornerMap[id] = std::move(ptr);

        return std::make_unique<Geometry3D::GeometryCollection3D>(
            std::move(surfaceMap), std::move(edgeMap), std::move(cornerMap));
    }

    std::unique_ptr<Topology3D::Topology3D> createTopology()
    {
        return std::make_unique<Topology3D::Topology3D>(topoSurfaces, topoEdges, topoCorners);
    }

    std::unordered_map<std::string, std::unique_ptr<Geometry3D::ICorner3D>> corners;
    std::unordered_map<std::string, std::unique_ptr<Geometry3D::IEdge3D>> edges;
    std::unordered_map<std::string, std::unique_ptr<Geometry3D::ISurface3D>> surfaces;

    std::unordered_map<std::string, Topology3D::Corner3D> topoCorners;
    std::unordered_map<std::string, Topology3D::Edge3D> topoEdges;
    std::unordered_map<std::string, Topology3D::Surface3D> topoSurfaces;
};

// Helper to create a triangle geometry and topology.
// Corners: c0=(0,0), c1=(1,0), c2=(0.5,1)
// Edges: e0=c0->c1, e1=c1->c2, e2=c2->c0
struct SimpleTriangleFixture
{
    SimpleTriangleFixture()
    {
        corners["c0"] = std::make_unique<MockCorner3D>("c0", Point3D(0, 0, 0));
        corners["c1"] = std::make_unique<MockCorner3D>("c1", Point3D(1, 0, 0));
        corners["c2"] = std::make_unique<MockCorner3D>("c2", Point3D(0.5, 1, 0));

        edges["e0"] = std::make_unique<MockEdge3D>("e0", Point3D(0, 0, 0), Point3D(1, 0, 0));
        edges["e1"] = std::make_unique<MockEdge3D>("e1", Point3D(1, 0, 0), Point3D(0.5, 1, 0));
        edges["e2"] = std::make_unique<MockEdge3D>("e2", Point3D(0.5, 1, 0), Point3D(0, 0, 0));

        surfaces["s0"] = std::make_unique<MockPlanarSurface>("s0");

        topoCorners.emplace("c0", Topology3D::Corner3D("c0", {"e0", "e2"}, {"s0"}));
        topoCorners.emplace("c1", Topology3D::Corner3D("c1", {"e0", "e1"}, {"s0"}));
        topoCorners.emplace("c2", Topology3D::Corner3D("c2", {"e1", "e2"}, {"s0"}));

        topoEdges.emplace("e0", Topology3D::Edge3D("e0", "c0", "c1", {"s0"}));
        topoEdges.emplace("e1", Topology3D::Edge3D("e1", "c1", "c2", {"s0"}));
        topoEdges.emplace("e2", Topology3D::Edge3D("e2", "c2", "c0", {"s0"}));

        topoSurfaces.emplace("s0", Topology3D::Surface3D(
            "s0",
            {"e0", "e1", "e2"},
            {"c0", "c1", "c2"}));
    }

    std::unique_ptr<Geometry3D::GeometryCollection3D> createGeometry()
    {
        std::unordered_map<std::string, std::unique_ptr<Geometry3D::ISurface3D>> surfaceMap;
        std::unordered_map<std::string, std::unique_ptr<Geometry3D::IEdge3D>> edgeMap;
        std::unordered_map<std::string, std::unique_ptr<Geometry3D::ICorner3D>> cornerMap;

        for (auto& [id, ptr] : surfaces) surfaceMap[id] = std::move(ptr);
        for (auto& [id, ptr] : edges) edgeMap[id] = std::move(ptr);
        for (auto& [id, ptr] : corners) cornerMap[id] = std::move(ptr);

        return std::make_unique<Geometry3D::GeometryCollection3D>(
            std::move(surfaceMap), std::move(edgeMap), std::move(cornerMap));
    }

    std::unique_ptr<Topology3D::Topology3D> createTopology()
    {
        return std::make_unique<Topology3D::Topology3D>(topoSurfaces, topoEdges, topoCorners);
    }

    std::unordered_map<std::string, std::unique_ptr<Geometry3D::ICorner3D>> corners;
    std::unordered_map<std::string, std::unique_ptr<Geometry3D::IEdge3D>> edges;
    std::unordered_map<std::string, std::unique_ptr<Geometry3D::ISurface3D>> surfaces;

    std::unordered_map<std::string, Topology3D::Corner3D> topoCorners;
    std::unordered_map<std::string, Topology3D::Edge3D> topoEdges;
    std::unordered_map<std::string, Topology3D::Surface3D> topoSurfaces;
};

// Build DiscretizationResult2D for the unit square (4 corner points only, local idx 0-3).
// Points: [c0=(0,0), c1=(1,0), c2=(1,1), c3=(0,1)]
DiscretizationResult2D makeSquareDisc2D()
{
    DiscretizationResult2D disc2D;
    disc2D.points = {Point2D(0, 0), Point2D(1, 0), Point2D(1, 1), Point2D(0, 1)};
    disc2D.tParameters.resize(4);
    disc2D.geometryIds.resize(4);
    disc2D.cornerIdToPointIndexMap = {{"c0", 0}, {"c1", 1}, {"c2", 2}, {"c3", 3}};
    disc2D.edgeIdToPointIndicesMap = {
        {"e0", {0, 1}}, {"e1", {1, 2}}, {"e2", {2, 3}}, {"e3", {3, 0}}};
    return disc2D;
}

// Build DiscretizationResult2D for the triangle (3 corner points only, local idx 0-2).
// Points: [c0=(0,0), c1=(1,0), c2=(0.5,1)]
DiscretizationResult2D makeTriangleDisc2D()
{
    DiscretizationResult2D disc2D;
    disc2D.points = {Point2D(0, 0), Point2D(1, 0), Point2D(0.5, 1)};
    disc2D.tParameters.resize(3);
    disc2D.geometryIds.resize(3);
    disc2D.cornerIdToPointIndexMap = {{"c0", 0}, {"c1", 1}, {"c2", 2}};
    disc2D.edgeIdToPointIndicesMap = {{"e0", {0, 1}}, {"e1", {1, 2}}, {"e2", {2, 0}}};
    return disc2D;
}

} // namespace

// ============================================================================
// FacetTriangulation Tests — square topology
// ============================================================================

class FacetTriangulationTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        fixture_ = std::make_unique<SimpleSquareFixture>();
        geometry_ = fixture_->createGeometry();
        topology_ = fixture_->createTopology();
    }

    std::unique_ptr<SimpleSquareFixture> fixture_;
    std::unique_ptr<Geometry3D::GeometryCollection3D> geometry_;
    std::unique_ptr<Topology3D::Topology3D> topology_;
};

TEST_F(FacetTriangulationTest, ConstructsWithValidSurface)
{
    auto* surface = geometry_->getSurface("s0");
    const auto& topoSurface = topology_->getSurface("s0");

    ASSERT_NE(surface, nullptr);

    FacetTriangulation facetTriang(*surface, topoSurface, *topology_, *geometry_);

    EXPECT_EQ(facetTriang.getSurfaceId(), "s0");
}

TEST_F(FacetTriangulationTest, InitializeCreatesTriangulation)
{
    auto* surface = geometry_->getSurface("s0");
    const auto& topoSurface = topology_->getSurface("s0");

    FacetTriangulation facetTriang(*surface, topoSurface, *topology_, *geometry_);

    auto disc2D = makeSquareDisc2D();
    std::vector<size_t> localIdxToNode3DId = {0, 1, 2, 3};

    facetTriang.initialize(disc2D, localIdxToNode3DId);

    // A unit square with 4 corner-only boundary produces 2 triangles
    auto subfacets = facetTriang.getSubfacets();
    EXPECT_EQ(subfacets.size(), 2u);
}

TEST_F(FacetTriangulationTest, GetSubfacetsReturns3DNodeIds)
{
    auto* surface = geometry_->getSurface("s0");
    const auto& topoSurface = topology_->getSurface("s0");

    FacetTriangulation facetTriang(*surface, topoSurface, *topology_, *geometry_);

    // Map local indices 0-3 to 3D node IDs 100-103
    auto disc2D = makeSquareDisc2D();
    std::vector<size_t> localIdxToNode3DId = {100, 101, 102, 103};

    facetTriang.initialize(disc2D, localIdxToNode3DId);

    auto subfacets = facetTriang.getSubfacets();

    // All subfacet node IDs should come from the 3D node IDs 100-103
    for (const auto& subfacet : subfacets)
    {
        EXPECT_TRUE(subfacet.nodeId1 >= 100 && subfacet.nodeId1 <= 103);
        EXPECT_TRUE(subfacet.nodeId2 >= 100 && subfacet.nodeId2 <= 103);
        EXPECT_TRUE(subfacet.nodeId3 >= 100 && subfacet.nodeId3 <= 103);
        EXPECT_EQ(subfacet.geometryId, "s0");
    }
}

// ============================================================================
// FacetTriangulation Tests — triangle topology
// ============================================================================

class TriangleFacetTriangulationTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        fixture_ = std::make_unique<SimpleTriangleFixture>();
        geometry_ = fixture_->createGeometry();
        topology_ = fixture_->createTopology();
    }

    std::unique_ptr<SimpleTriangleFixture> fixture_;
    std::unique_ptr<Geometry3D::GeometryCollection3D> geometry_;
    std::unique_ptr<Topology3D::Topology3D> topology_;
};

TEST_F(TriangleFacetTriangulationTest, NodeMappingIsBidirectional)
{
    auto* surface = geometry_->getSurface("s0");
    const auto& topoSurface = topology_->getSurface("s0");

    FacetTriangulation facetTriang(*surface, topoSurface, *topology_, *geometry_);

    auto disc2D = makeTriangleDisc2D();
    std::vector<size_t> localIdxToNode3DId = {10, 20, 30};

    facetTriang.initialize(disc2D, localIdxToNode3DId);

    // Check that we can map 3D -> 2D -> 3D
    for (size_t node3D : {10, 20, 30})
    {
        auto node2D = facetTriang.get2DNodeId(node3D);
        ASSERT_TRUE(node2D.has_value());

        auto backTo3D = facetTriang.get3DNodeId(*node2D);
        ASSERT_TRUE(backTo3D.has_value());
        EXPECT_EQ(*backTo3D, node3D);
    }
}

TEST_F(TriangleFacetTriangulationTest, InsertVertexAddsToTriangulation)
{
    auto* surface = geometry_->getSurface("s0");
    const auto& topoSurface = topology_->getSurface("s0");

    FacetTriangulation facetTriang(*surface, topoSurface, *topology_, *geometry_);

    auto disc2D = makeTriangleDisc2D();
    std::vector<size_t> localIdxToNode3DId = {0, 1, 2};

    facetTriang.initialize(disc2D, localIdxToNode3DId);

    auto initialSubfacets = facetTriang.getSubfacets();
    EXPECT_EQ(initialSubfacets.size(), 1u);

    // Insert a vertex near the centroid
    bool inserted = facetTriang.insertVertex(99, Point2D(0.5, 0.33));
    EXPECT_TRUE(inserted);

    // Should now have more triangles
    auto finalSubfacets = facetTriang.getSubfacets();
    EXPECT_GT(finalSubfacets.size(), 1u);

    // The new node should be mappable
    auto node2D = facetTriang.get2DNodeId(99);
    EXPECT_TRUE(node2D.has_value());
}

TEST_F(TriangleFacetTriangulationTest, InsertVertexRejectsExistingNode)
{
    auto* surface = geometry_->getSurface("s0");
    const auto& topoSurface = topology_->getSurface("s0");

    FacetTriangulation facetTriang(*surface, topoSurface, *topology_, *geometry_);

    auto disc2D = makeTriangleDisc2D();
    std::vector<size_t> localIdxToNode3DId = {0, 1, 2};

    facetTriang.initialize(disc2D, localIdxToNode3DId);

    // Try to insert with an existing node ID — should succeed but not add duplicate
    bool inserted = facetTriang.insertVertex(0, Point2D(0.5, 0.5));
    EXPECT_TRUE(inserted);

    // Subfacet count should not have changed
    auto subfacets = facetTriang.getSubfacets();
    EXPECT_EQ(subfacets.size(), 1u);
}

TEST_F(TriangleFacetTriangulationTest, GetNodeIdReturnsNulloptForUnknownNode)
{
    auto* surface = geometry_->getSurface("s0");
    const auto& topoSurface = topology_->getSurface("s0");

    FacetTriangulation facetTriang(*surface, topoSurface, *topology_, *geometry_);

    auto disc2D = makeTriangleDisc2D();
    std::vector<size_t> localIdxToNode3DId = {0, 1, 2};

    facetTriang.initialize(disc2D, localIdxToNode3DId);

    // Unknown 3D node ID
    auto node2D = facetTriang.get2DNodeId(999);
    EXPECT_FALSE(node2D.has_value());

    // Unknown 2D node ID
    auto node3D = facetTriang.get3DNodeId(999);
    EXPECT_FALSE(node3D.has_value());
}
