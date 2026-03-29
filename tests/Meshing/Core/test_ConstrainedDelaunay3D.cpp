#include <gtest/gtest.h>

#include "Common/BoundingBox2D.h"
#include "Common/Types.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ICorner3D.h"
#include "Geometry/3D/Base/IEdge3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Core/3D/Volume/ConstrainedDelaunay3D.h"
#include "Meshing/Core/3D/General/DiscretizationResult3D.h"
#include "Meshing/Core/3D/General/GeometryStructures3D.h"
#include "Meshing/Core/3D/General/MeshingContext3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
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

class MockPlanarSurface : public Geometry3D::ISurface3D
{
public:
    MockPlanarSurface(const std::string& id, double z = 0.0) :
        id_(id),
        z_(z)
    {
    }

    std::array<double, 3> getNormal(double /*u*/, double /*v*/) const override
    {
        return {0.0, 0.0, 1.0};
    }

    Point3D getPoint(double u, double v) const override
    {
        return Point3D(u, v, z_);
    }

    Common::BoundingBox2D getParameterBounds() const override
    {
        return Common::BoundingBox2D(0.0, 1.0, 0.0, 1.0);
    }

    double getGap(const Point3D& point) const override
    {
        return std::abs(point.z() - z_);
    }

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
    double z_;
};

// Creates a unit cube geometry and topology
struct UnitCubeFixture
{
    UnitCubeFixture()
    {
        // 8 corners of unit cube
        Point3D c0_pt(0, 0, 0);
        Point3D c1_pt(1, 0, 0);
        Point3D c2_pt(1, 1, 0);
        Point3D c3_pt(0, 1, 0);
        Point3D c4_pt(0, 0, 1);
        Point3D c5_pt(1, 0, 1);
        Point3D c6_pt(1, 1, 1);
        Point3D c7_pt(0, 1, 1);

        corners["c0"] = std::make_unique<MockCorner3D>("c0", c0_pt);
        corners["c1"] = std::make_unique<MockCorner3D>("c1", c1_pt);
        corners["c2"] = std::make_unique<MockCorner3D>("c2", c2_pt);
        corners["c3"] = std::make_unique<MockCorner3D>("c3", c3_pt);
        corners["c4"] = std::make_unique<MockCorner3D>("c4", c4_pt);
        corners["c5"] = std::make_unique<MockCorner3D>("c5", c5_pt);
        corners["c6"] = std::make_unique<MockCorner3D>("c6", c6_pt);
        corners["c7"] = std::make_unique<MockCorner3D>("c7", c7_pt);

        // 12 edges
        edges["e0"] = std::make_unique<MockEdge3D>("e0", c0_pt, c1_pt);
        edges["e1"] = std::make_unique<MockEdge3D>("e1", c1_pt, c2_pt);
        edges["e2"] = std::make_unique<MockEdge3D>("e2", c2_pt, c3_pt);
        edges["e3"] = std::make_unique<MockEdge3D>("e3", c3_pt, c0_pt);
        edges["e4"] = std::make_unique<MockEdge3D>("e4", c4_pt, c5_pt);
        edges["e5"] = std::make_unique<MockEdge3D>("e5", c5_pt, c6_pt);
        edges["e6"] = std::make_unique<MockEdge3D>("e6", c6_pt, c7_pt);
        edges["e7"] = std::make_unique<MockEdge3D>("e7", c7_pt, c4_pt);
        edges["e8"] = std::make_unique<MockEdge3D>("e8", c0_pt, c4_pt);
        edges["e9"] = std::make_unique<MockEdge3D>("e9", c1_pt, c5_pt);
        edges["e10"] = std::make_unique<MockEdge3D>("e10", c2_pt, c6_pt);
        edges["e11"] = std::make_unique<MockEdge3D>("e11", c3_pt, c7_pt);

        // 6 surfaces (bottom z=0 only for simplicity in testing)
        surfaces["bottom"] = std::make_unique<MockPlanarSurface>("bottom", 0.0);

        // Topology corners
        topoCorners.emplace("c0", Topology3D::Corner3D("c0", {"e0", "e3", "e8"}, {"bottom"}));
        topoCorners.emplace("c1", Topology3D::Corner3D("c1", {"e0", "e1", "e9"}, {"bottom"}));
        topoCorners.emplace("c2", Topology3D::Corner3D("c2", {"e1", "e2", "e10"}, {"bottom"}));
        topoCorners.emplace("c3", Topology3D::Corner3D("c3", {"e2", "e3", "e11"}, {"bottom"}));
        topoCorners.emplace("c4", Topology3D::Corner3D("c4", {"e4", "e7", "e8"}, {}));
        topoCorners.emplace("c5", Topology3D::Corner3D("c5", {"e4", "e5", "e9"}, {}));
        topoCorners.emplace("c6", Topology3D::Corner3D("c6", {"e5", "e6", "e10"}, {}));
        topoCorners.emplace("c7", Topology3D::Corner3D("c7", {"e6", "e7", "e11"}, {}));

        // Topology edges (only bottom face edges)
        topoEdges.emplace("e0", Topology3D::Edge3D("e0", "c0", "c1", {"bottom"}));
        topoEdges.emplace("e1", Topology3D::Edge3D("e1", "c1", "c2", {"bottom"}));
        topoEdges.emplace("e2", Topology3D::Edge3D("e2", "c2", "c3", {"bottom"}));
        topoEdges.emplace("e3", Topology3D::Edge3D("e3", "c3", "c0", {"bottom"}));

        // Bottom surface
        topoSurfaces.emplace("bottom", Topology3D::Surface3D(
            "bottom", {"e0", "e1", "e2", "e3"}, {"c0", "c1", "c2", "c3"}));
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

        // Add all 8 corners of the cube for proper 3D tetrahedralization
        result.points.push_back(Point3D(0, 0, 0)); // 0 -> c0
        result.points.push_back(Point3D(1, 0, 0)); // 1 -> c1
        result.points.push_back(Point3D(1, 1, 0)); // 2 -> c2
        result.points.push_back(Point3D(0, 1, 0)); // 3 -> c3
        result.points.push_back(Point3D(0, 0, 1)); // 4 -> c4
        result.points.push_back(Point3D(1, 0, 1)); // 5 -> c5
        result.points.push_back(Point3D(1, 1, 1)); // 6 -> c6
        result.points.push_back(Point3D(0, 1, 1)); // 7 -> c7

        result.edgeParameters = {{}, {}, {}, {}, {}, {}, {}, {}};
        result.geometryIds = {{"c0"}, {"c1"}, {"c2"}, {"c3"}, {"c4"}, {"c5"}, {"c6"}, {"c7"}};

        result.cornerIdToPointIndexMap["c0"] = 0;
        result.cornerIdToPointIndexMap["c1"] = 1;
        result.cornerIdToPointIndexMap["c2"] = 2;
        result.cornerIdToPointIndexMap["c3"] = 3;
        result.cornerIdToPointIndexMap["c4"] = 4;
        result.cornerIdToPointIndexMap["c5"] = 5;
        result.cornerIdToPointIndexMap["c6"] = 6;
        result.cornerIdToPointIndexMap["c7"] = 7;

        // Bottom face edges
        result.edgeIdToPointIndicesMap["e0"] = {0, 1};
        result.edgeIdToPointIndicesMap["e1"] = {1, 2};
        result.edgeIdToPointIndicesMap["e2"] = {2, 3};
        result.edgeIdToPointIndicesMap["e3"] = {3, 0};

        result.surfaceIdToPointIndicesMap["bottom"] = {};

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
// ConstrainedDelaunay3D Tests
// ============================================================================

class ConstrainedDelaunay3DTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        fixture_ = std::make_unique<UnitCubeFixture>();
        geometry_ = fixture_->createGeometry();
        topology_ = fixture_->createTopology();
    }

    std::unique_ptr<UnitCubeFixture> fixture_;
    std::unique_ptr<Geometry3D::GeometryCollection3D> geometry_;
    std::unique_ptr<Topology3D::Topology3D> topology_;
};

TEST_F(ConstrainedDelaunay3DTest, ConstructsWithContextAndDiscretization)
{
    MeshingContext3D context(*geometry_, *topology_);
    auto discretization = fixture_->createDiscretization();

    ConstrainedDelaunay3D cd3d(context, discretization);

    // Should construct without error
    SUCCEED();
}

TEST_F(ConstrainedDelaunay3DTest, TetrahedralizeCreatesTetrahedra)
{
    MeshingContext3D context(*geometry_, *topology_);
    auto discretization = fixture_->createDiscretization();

    ConstrainedDelaunay3D cd3d(context, discretization);
    cd3d.tetrahedralize();

    const auto& meshData = context.getMeshData();

    // Should have created nodes and tetrahedra
    EXPECT_GE(meshData.getNodeCount(), 4); // At least the 4 corner nodes
    EXPECT_GE(meshData.getElementCount(), 1);
}

TEST_F(ConstrainedDelaunay3DTest, TetrahedralizeExtractsSubsegments)
{
    MeshingContext3D context(*geometry_, *topology_);
    auto discretization = fixture_->createDiscretization();

    ConstrainedDelaunay3D cd3d(context, discretization);
    cd3d.tetrahedralize();

    const auto& meshData = context.getMeshData();
    const auto& subsegments = meshData.getConstrainedSubsegments();

    // Should have 4 subsegments for the 4 edges of the bottom face
    EXPECT_EQ(subsegments.size(), 4);
}

TEST_F(ConstrainedDelaunay3DTest, TetrahedralizeCreatesSubfacets)
{
    MeshingContext3D context(*geometry_, *topology_);
    auto discretization = fixture_->createDiscretization();

    ConstrainedDelaunay3D cd3d(context, discretization);
    cd3d.tetrahedralize();

    const auto& meshData = context.getMeshData();
    const auto& subfacets = meshData.getConstrainedSubfacets();

    // Bottom face should be triangulated into 2 subfacets
    EXPECT_EQ(subfacets.size(), 2);
}

TEST_F(ConstrainedDelaunay3DTest, SubsegmentsHaveCorrectGeometryIds)
{
    MeshingContext3D context(*geometry_, *topology_);
    auto discretization = fixture_->createDiscretization();

    ConstrainedDelaunay3D cd3d(context, discretization);
    cd3d.tetrahedralize();

    const auto& meshData = context.getMeshData();
    const auto& subsegments = meshData.getConstrainedSubsegments();

    std::set<std::string> expectedEdgeIds = {"e0", "e1", "e2", "e3"};
    std::set<std::string> foundEdgeIds;

    for (const auto& subsegment : subsegments)
    {
        foundEdgeIds.insert(subsegment.geometryId);
    }

    EXPECT_EQ(foundEdgeIds, expectedEdgeIds);
}

TEST_F(ConstrainedDelaunay3DTest, SubfacetsHaveCorrectGeometryId)
{
    MeshingContext3D context(*geometry_, *topology_);
    auto discretization = fixture_->createDiscretization();

    ConstrainedDelaunay3D cd3d(context, discretization);
    cd3d.tetrahedralize();

    const auto& meshData = context.getMeshData();
    const auto& subfacets = meshData.getConstrainedSubfacets();

    for (const auto& subfacet : subfacets)
    {
        EXPECT_EQ(subfacet.geometryId, "bottom");
    }
}

TEST_F(ConstrainedDelaunay3DTest, GetPointIndexToNodeIdMapReturnsValidMapping)
{
    MeshingContext3D context(*geometry_, *topology_);
    auto discretization = fixture_->createDiscretization();

    ConstrainedDelaunay3D cd3d(context, discretization);
    cd3d.tetrahedralize();

    const auto& pointIndexToNodeIdMap = cd3d.getPointIndexToNodeIdMap();

    // Should have mapping for all 8 corner points
    EXPECT_EQ(pointIndexToNodeIdMap.size(), 8);

    // All node IDs should be valid
    const auto& meshData = context.getMeshData();
    for (const auto& [pointIdx, nodeId] : pointIndexToNodeIdMap)
    {
        EXPECT_NE(meshData.getNode(nodeId), nullptr)
            << "Node ID " << nodeId << " should exist in mesh";
    }
}

TEST_F(ConstrainedDelaunay3DTest, GetFacetManagerReturnsValidManager)
{
    MeshingContext3D context(*geometry_, *topology_);
    auto discretization = fixture_->createDiscretization();

    ConstrainedDelaunay3D cd3d(context, discretization);
    cd3d.tetrahedralize();

    const auto& facetManager = cd3d.getFacetManager();

    // Should have 1 facet triangulation (bottom face)
    EXPECT_EQ(facetManager.size(), 1);

    // Should be able to get the bottom face triangulation
    auto* bottomTriang = facetManager.getFacetTriangulation("bottom");
    EXPECT_NE(bottomTriang, nullptr);
}

TEST_F(ConstrainedDelaunay3DTest, SubsegmentNodeIdsAreValidMeshNodes)
{
    MeshingContext3D context(*geometry_, *topology_);
    auto discretization = fixture_->createDiscretization();

    ConstrainedDelaunay3D cd3d(context, discretization);
    cd3d.tetrahedralize();

    const auto& meshData = context.getMeshData();
    const auto& subsegments = meshData.getConstrainedSubsegments();

    for (const auto& subsegment : subsegments)
    {
        EXPECT_NE(meshData.getNode(subsegment.nodeId1), nullptr)
            << "Subsegment node ID " << subsegment.nodeId1 << " should exist";
        EXPECT_NE(meshData.getNode(subsegment.nodeId2), nullptr)
            << "Subsegment node ID " << subsegment.nodeId2 << " should exist";
    }
}

TEST_F(ConstrainedDelaunay3DTest, SubfacetNodeIdsAreValidMeshNodes)
{
    MeshingContext3D context(*geometry_, *topology_);
    auto discretization = fixture_->createDiscretization();

    ConstrainedDelaunay3D cd3d(context, discretization);
    cd3d.tetrahedralize();

    const auto& meshData = context.getMeshData();
    const auto& subfacets = meshData.getConstrainedSubfacets();

    for (const auto& subfacet : subfacets)
    {
        EXPECT_NE(meshData.getNode(subfacet.nodeId1), nullptr)
            << "Subfacet node ID " << subfacet.nodeId1 << " should exist";
        EXPECT_NE(meshData.getNode(subfacet.nodeId2), nullptr)
            << "Subfacet node ID " << subfacet.nodeId2 << " should exist";
        EXPECT_NE(meshData.getNode(subfacet.nodeId3), nullptr)
            << "Subfacet node ID " << subfacet.nodeId3 << " should exist";
    }
}

TEST_F(ConstrainedDelaunay3DTest, TetrahedralizationIsUnconstrainedDelaunay)
{
    // Per Shewchuk's algorithm, the tetrahedralization is unconstrained.
    // Some subsegments/subfacets may NOT exist as mesh edges/faces.
    // This is expected behavior - constraints are recovered through refinement.

    MeshingContext3D context(*geometry_, *topology_);
    auto discretization = fixture_->createDiscretization();

    ConstrainedDelaunay3D cd3d(context, discretization);
    cd3d.tetrahedralize();

    const auto& meshData = context.getMeshData();

    // The mesh should exist and be valid
    EXPECT_GT(meshData.getNodeCount(), 0);
    EXPECT_GT(meshData.getElementCount(), 0);

    // Note: We don't check if all constraints exist in the mesh because
    // this is an unconstrained Delaunay tetrahedralization.
    // Constraints are tracked for later recovery, not enforced.
    SUCCEED();
}
