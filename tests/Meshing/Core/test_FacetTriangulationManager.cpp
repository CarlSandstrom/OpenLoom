#include <gtest/gtest.h>

#include "Common/BoundingBox2D.h"
#include "Common/Types.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ICorner3D.h"
#include "Geometry/3D/Base/IEdge3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Core/3D/DiscretizationResult3D.h"
#include "Meshing/Core/3D/FacetTriangulationManager.h"
#include "Meshing/Core/3D/GeometryStructures3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Topology/Corner3D.h"
#include "Topology/Edge3D.h"
#include "Topology/Surface3D.h"
#include "Topology/Topology3D.h"

using namespace Meshing;

// ============================================================================
// Mock Geometry Classes for Testing (same as in test_FacetTriangulation.cpp)
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
        return Point2D(point.x(), point.y());
    }

    std::string getId() const override { return id_; }

private:
    std::string id_;
};

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
            "s0", {"e0", "e1", "e2", "e3"}, {"c0", "c1", "c2", "c3"}));
    }

    std::unique_ptr<Geometry3D::GeometryCollection3D> createGeometry()
    {
        std::unordered_map<std::string, std::unique_ptr<Geometry3D::ISurface3D>> surfaceMap;
        std::unordered_map<std::string, std::unique_ptr<Geometry3D::IEdge3D>> edgeMap;
        std::unordered_map<std::string, std::unique_ptr<Geometry3D::ICorner3D>> cornerMap;

        for (auto& [id, ptr] : surfaces)
            surfaceMap[id] = std::move(ptr);
        for (auto& [id, ptr] : edges)
            edgeMap[id] = std::move(ptr);
        for (auto& [id, ptr] : corners)
            cornerMap[id] = std::move(ptr);

        return std::make_unique<Geometry3D::GeometryCollection3D>(
            std::move(surfaceMap), std::move(edgeMap), std::move(cornerMap));
    }

    std::unique_ptr<Topology3D::Topology3D> createTopology()
    {
        return std::make_unique<Topology3D::Topology3D>(topoSurfaces, topoEdges, topoCorners);
    }

    DiscretizationResult3D createDiscretization()
    {
        DiscretizationResult3D result;

        // Add corner points
        result.points.push_back(Point3D(0, 0, 0)); // index 0 -> c0
        result.points.push_back(Point3D(1, 0, 0)); // index 1 -> c1
        result.points.push_back(Point3D(1, 1, 0)); // index 2 -> c2
        result.points.push_back(Point3D(0, 1, 0)); // index 3 -> c3

        result.edgeParameters = {{}, {}, {}, {}};
        result.geometryIds = {{"c0"}, {"c1"}, {"c2"}, {"c3"}};

        result.cornerIdToPointIndexMap["c0"] = 0;
        result.cornerIdToPointIndexMap["c1"] = 1;
        result.cornerIdToPointIndexMap["c2"] = 2;
        result.cornerIdToPointIndexMap["c3"] = 3;

        // Edge point indices (corners only, no intermediate points)
        result.edgeIdToPointIndicesMap["e0"] = {0, 1};
        result.edgeIdToPointIndicesMap["e1"] = {1, 2};
        result.edgeIdToPointIndicesMap["e2"] = {2, 3};
        result.edgeIdToPointIndicesMap["e3"] = {3, 0};

        // No interior surface points
        result.surfaceIdToPointIndicesMap["s0"] = {};

        return result;
    }

    std::unordered_map<std::string, std::unique_ptr<Geometry3D::ICorner3D>> corners;
    std::unordered_map<std::string, std::unique_ptr<Geometry3D::IEdge3D>> edges;
    std::unordered_map<std::string, std::unique_ptr<Geometry3D::ISurface3D>> surfaces;

    std::unordered_map<std::string, Topology3D::Corner3D> topoCorners;
    std::unordered_map<std::string, Topology3D::Edge3D> topoEdges;
    std::unordered_map<std::string, Topology3D::Surface3D> topoSurfaces;
};

} // namespace

// ============================================================================
// FacetTriangulationManager Tests
// ============================================================================

class FacetTriangulationManagerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        fixture_ = std::make_unique<SimpleSquareFixture>();
        geometry_ = fixture_->createGeometry();
        topology_ = fixture_->createTopology();
        mutator_ = std::make_unique<MeshMutator3D>(meshData_);
    }

    size_t addNode(double x, double y, double z)
    {
        return mutator_->addNode(Point3D(x, y, z));
    }

    std::unique_ptr<SimpleSquareFixture> fixture_;
    std::unique_ptr<Geometry3D::GeometryCollection3D> geometry_;
    std::unique_ptr<Topology3D::Topology3D> topology_;
    MeshData3D meshData_;
    std::unique_ptr<MeshMutator3D> mutator_;
};

TEST_F(FacetTriangulationManagerTest, ConstructsWithGeometryAndTopology)
{
    FacetTriangulationManager manager(*geometry_, *topology_);
    EXPECT_EQ(manager.size(), 0); // Not initialized yet
}

TEST_F(FacetTriangulationManagerTest, InitializeFromDiscretizationCreatesFacetTriangulations)
{
    FacetTriangulationManager manager(*geometry_, *topology_);

    // Create mesh nodes corresponding to discretization points
    size_t n0 = addNode(0, 0, 0);
    size_t n1 = addNode(1, 0, 0);
    size_t n2 = addNode(1, 1, 0);
    size_t n3 = addNode(0, 1, 0);

    // Create point index to node ID map
    std::map<size_t, size_t> pointIndexToNodeIdMap;
    pointIndexToNodeIdMap[0] = n0;
    pointIndexToNodeIdMap[1] = n1;
    pointIndexToNodeIdMap[2] = n2;
    pointIndexToNodeIdMap[3] = n3;

    auto discretization = fixture_->createDiscretization();

    manager.initializeFromDiscretization(discretization, pointIndexToNodeIdMap, meshData_);

    EXPECT_EQ(manager.size(), 1); // One surface
}

TEST_F(FacetTriangulationManagerTest, GetFacetTriangulationReturnsValidPointer)
{
    FacetTriangulationManager manager(*geometry_, *topology_);

    size_t n0 = addNode(0, 0, 0);
    size_t n1 = addNode(1, 0, 0);
    size_t n2 = addNode(1, 1, 0);
    size_t n3 = addNode(0, 1, 0);

    std::map<size_t, size_t> pointIndexToNodeIdMap = {{0, n0}, {1, n1}, {2, n2}, {3, n3}};
    auto discretization = fixture_->createDiscretization();

    manager.initializeFromDiscretization(discretization, pointIndexToNodeIdMap, meshData_);

    auto* facetTriang = manager.getFacetTriangulation("s0");
    ASSERT_NE(facetTriang, nullptr);
    EXPECT_EQ(facetTriang->getSurfaceId(), "s0");
}

TEST_F(FacetTriangulationManagerTest, GetFacetTriangulationReturnsNullForUnknownSurface)
{
    FacetTriangulationManager manager(*geometry_, *topology_);

    size_t n0 = addNode(0, 0, 0);
    size_t n1 = addNode(1, 0, 0);
    size_t n2 = addNode(1, 1, 0);
    size_t n3 = addNode(0, 1, 0);

    std::map<size_t, size_t> pointIndexToNodeIdMap = {{0, n0}, {1, n1}, {2, n2}, {3, n3}};
    auto discretization = fixture_->createDiscretization();

    manager.initializeFromDiscretization(discretization, pointIndexToNodeIdMap, meshData_);

    auto* facetTriang = manager.getFacetTriangulation("unknown_surface");
    EXPECT_EQ(facetTriang, nullptr);
}

TEST_F(FacetTriangulationManagerTest, GetAllSubfacetsReturnsCorrectCount)
{
    FacetTriangulationManager manager(*geometry_, *topology_);

    size_t n0 = addNode(0, 0, 0);
    size_t n1 = addNode(1, 0, 0);
    size_t n2 = addNode(1, 1, 0);
    size_t n3 = addNode(0, 1, 0);

    std::map<size_t, size_t> pointIndexToNodeIdMap = {{0, n0}, {1, n1}, {2, n2}, {3, n3}};
    auto discretization = fixture_->createDiscretization();

    manager.initializeFromDiscretization(discretization, pointIndexToNodeIdMap, meshData_);

    auto subfacets = manager.getAllSubfacets();

    // A square should be triangulated into 2 triangles
    EXPECT_EQ(subfacets.size(), 2);
}

TEST_F(FacetTriangulationManagerTest, GetSubfacetsForSurfaceReturnsCorrectSubfacets)
{
    FacetTriangulationManager manager(*geometry_, *topology_);

    size_t n0 = addNode(0, 0, 0);
    size_t n1 = addNode(1, 0, 0);
    size_t n2 = addNode(1, 1, 0);
    size_t n3 = addNode(0, 1, 0);

    std::map<size_t, size_t> pointIndexToNodeIdMap = {{0, n0}, {1, n1}, {2, n2}, {3, n3}};
    auto discretization = fixture_->createDiscretization();

    manager.initializeFromDiscretization(discretization, pointIndexToNodeIdMap, meshData_);

    auto subfacets = manager.getSubfacetsForSurface("s0");
    EXPECT_EQ(subfacets.size(), 2);

    // All subfacets should have surface ID "s0"
    for (const auto& subfacet : subfacets)
    {
        EXPECT_EQ(subfacet.geometryId, "s0");
    }
}

TEST_F(FacetTriangulationManagerTest, GetSubfacetsForUnknownSurfaceReturnsEmpty)
{
    FacetTriangulationManager manager(*geometry_, *topology_);

    size_t n0 = addNode(0, 0, 0);
    size_t n1 = addNode(1, 0, 0);
    size_t n2 = addNode(1, 1, 0);
    size_t n3 = addNode(0, 1, 0);

    std::map<size_t, size_t> pointIndexToNodeIdMap = {{0, n0}, {1, n1}, {2, n2}, {3, n3}};
    auto discretization = fixture_->createDiscretization();

    manager.initializeFromDiscretization(discretization, pointIndexToNodeIdMap, meshData_);

    auto subfacets = manager.getSubfacetsForSurface("unknown");
    EXPECT_TRUE(subfacets.empty());
}

TEST_F(FacetTriangulationManagerTest, InsertVertexOnSurfaceAddsVertex)
{
    FacetTriangulationManager manager(*geometry_, *topology_);

    size_t n0 = addNode(0, 0, 0);
    size_t n1 = addNode(1, 0, 0);
    size_t n2 = addNode(1, 1, 0);
    size_t n3 = addNode(0, 1, 0);

    std::map<size_t, size_t> pointIndexToNodeIdMap = {{0, n0}, {1, n1}, {2, n2}, {3, n3}};
    auto discretization = fixture_->createDiscretization();

    manager.initializeFromDiscretization(discretization, pointIndexToNodeIdMap, meshData_);

    auto initialSubfacets = manager.getAllSubfacets();
    size_t initialCount = initialSubfacets.size();

    // Insert a vertex in the middle of the surface
    size_t newNodeId = addNode(0.5, 0.5, 0);
    bool inserted = manager.insertVertexOnSurface(newNodeId, Point3D(0.5, 0.5, 0), "s0");
    EXPECT_TRUE(inserted);

    auto finalSubfacets = manager.getAllSubfacets();
    EXPECT_GT(finalSubfacets.size(), initialCount);
}

TEST_F(FacetTriangulationManagerTest, InsertVertexOnUnknownSurfaceFails)
{
    FacetTriangulationManager manager(*geometry_, *topology_);

    size_t n0 = addNode(0, 0, 0);
    size_t n1 = addNode(1, 0, 0);
    size_t n2 = addNode(1, 1, 0);
    size_t n3 = addNode(0, 1, 0);

    std::map<size_t, size_t> pointIndexToNodeIdMap = {{0, n0}, {1, n1}, {2, n2}, {3, n3}};
    auto discretization = fixture_->createDiscretization();

    manager.initializeFromDiscretization(discretization, pointIndexToNodeIdMap, meshData_);

    size_t newNodeId = addNode(0.5, 0.5, 0);
    bool inserted = manager.insertVertexOnSurface(newNodeId, Point3D(0.5, 0.5, 0), "unknown");
    EXPECT_FALSE(inserted);
}

TEST_F(FacetTriangulationManagerTest, SubfacetsUseCorrect3DNodeIds)
{
    FacetTriangulationManager manager(*geometry_, *topology_);

    // Use specific node IDs
    size_t n0 = addNode(0, 0, 0); // Should be 0
    size_t n1 = addNode(1, 0, 0); // Should be 1
    size_t n2 = addNode(1, 1, 0); // Should be 2
    size_t n3 = addNode(0, 1, 0); // Should be 3

    std::map<size_t, size_t> pointIndexToNodeIdMap = {{0, n0}, {1, n1}, {2, n2}, {3, n3}};
    auto discretization = fixture_->createDiscretization();

    manager.initializeFromDiscretization(discretization, pointIndexToNodeIdMap, meshData_);

    auto subfacets = manager.getAllSubfacets();

    // Verify all node IDs in subfacets are valid mesh node IDs
    std::set<size_t> validNodeIds = {n0, n1, n2, n3};
    for (const auto& subfacet : subfacets)
    {
        EXPECT_TRUE(validNodeIds.count(subfacet.nodeId1) > 0)
            << "Invalid node ID: " << subfacet.nodeId1;
        EXPECT_TRUE(validNodeIds.count(subfacet.nodeId2) > 0)
            << "Invalid node ID: " << subfacet.nodeId2;
        EXPECT_TRUE(validNodeIds.count(subfacet.nodeId3) > 0)
            << "Invalid node ID: " << subfacet.nodeId3;
    }
}
