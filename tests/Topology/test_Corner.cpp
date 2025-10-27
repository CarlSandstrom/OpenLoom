#include "../../Topology/Corner.h"
#include <gtest/gtest.h>
#include <set>
#include <string>

using namespace Topology;

class TopologyCornerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Setup common test data
        cornerId_ = "corner_001";
        edgeIds_ = {"edge_001", "edge_002", "edge_003"};
        surfaceIds_ = {"surface_001", "surface_002"};

        emptyEdgeIds_ = {};
        emptySurfaceIds_ = {};
    }

    void TearDown() override
    {
        // Cleanup
    }

    // Test data
    std::string cornerId_;
    std::set<std::string> edgeIds_;
    std::set<std::string> surfaceIds_;
    std::set<std::string> emptyEdgeIds_;
    std::set<std::string> emptySurfaceIds_;
};

TEST_F(TopologyCornerTest, ConstructorWithValidParameters)
{
    // Test Corner construction with valid parameters
    Topology::Corner corner(cornerId_, edgeIds_, surfaceIds_);

    EXPECT_EQ(corner.getId(), cornerId_);
    EXPECT_EQ(corner.getConnectedEdgeIds(), edgeIds_);
    EXPECT_EQ(corner.getConnectedSurfaceIds(), surfaceIds_);
}

TEST_F(TopologyCornerTest, ConstructorWithEmptyConnections)
{
    // Test Corner construction with empty connection sets
    Topology::Corner corner(cornerId_, emptyEdgeIds_, emptySurfaceIds_);

    EXPECT_EQ(corner.getId(), cornerId_);
    EXPECT_TRUE(corner.getConnectedEdgeIds().empty());
    EXPECT_TRUE(corner.getConnectedSurfaceIds().empty());
}

TEST_F(TopologyCornerTest, ConstructorWithEmptyId)
{
    // Test Corner construction with empty ID
    std::string emptyId = "";
    Topology::Corner corner(emptyId, edgeIds_, surfaceIds_);

    EXPECT_EQ(corner.getId(), emptyId);
    EXPECT_EQ(corner.getConnectedEdgeIds(), edgeIds_);
    EXPECT_EQ(corner.getConnectedSurfaceIds(), surfaceIds_);
}

TEST_F(TopologyCornerTest, GetIdReturnsCorrectValue)
{
    // Test that getId() returns the correct ID
    Topology::Corner corner(cornerId_, edgeIds_, surfaceIds_);

    EXPECT_EQ(corner.getId(), cornerId_);
}

TEST_F(TopologyCornerTest, GetConnectedEdgeIdsReturnsCorrectSet)
{
    // Test that getConnectedEdgeIds() returns the correct edge set
    Topology::Corner corner(cornerId_, edgeIds_, surfaceIds_);

    const auto& returnedEdgeIds = corner.getConnectedEdgeIds();
    EXPECT_EQ(returnedEdgeIds.size(), edgeIds_.size());

    for (const auto& edgeId : edgeIds_)
    {
        EXPECT_TRUE(returnedEdgeIds.find(edgeId) != returnedEdgeIds.end());
    }
}

TEST_F(TopologyCornerTest, GetConnectedSurfaceIdsReturnsCorrectSet)
{
    // Test that getConnectedSurfaceIds() returns the correct surface set
    Topology::Corner corner(cornerId_, edgeIds_, surfaceIds_);

    const auto& returnedSurfaceIds = corner.getConnectedSurfaceIds();
    EXPECT_EQ(returnedSurfaceIds.size(), surfaceIds_.size());

    for (const auto& surfaceId : surfaceIds_)
    {
        EXPECT_TRUE(returnedSurfaceIds.find(surfaceId) != returnedSurfaceIds.end());
    }
}

TEST_F(TopologyCornerTest, ConnectedEdgeIdsAreImmutable)
{
    // Test that the returned edge IDs set cannot be modified from outside
    Topology::Corner corner(cornerId_, edgeIds_, surfaceIds_);

    const auto& returnedEdgeIds = corner.getConnectedEdgeIds();
    size_t originalSize = returnedEdgeIds.size();

    // The reference should be const, preventing modification
    // This test verifies the const-correctness of the getter
    EXPECT_EQ(returnedEdgeIds.size(), originalSize);
    EXPECT_EQ(returnedEdgeIds, edgeIds_);
}

TEST_F(TopologyCornerTest, ConnectedSurfaceIdsAreImmutable)
{
    // Test that the returned surface IDs set cannot be modified from outside
    Topology::Corner corner(cornerId_, edgeIds_, surfaceIds_);

    const auto& returnedSurfaceIds = corner.getConnectedSurfaceIds();
    size_t originalSize = returnedSurfaceIds.size();

    // The reference should be const, preventing modification
    // This test verifies the const-correctness of the getter
    EXPECT_EQ(returnedSurfaceIds.size(), originalSize);
    EXPECT_EQ(returnedSurfaceIds, surfaceIds_);
}

TEST_F(TopologyCornerTest, SingleEdgeConnection)
{
    // Test Corner with a single edge connection
    std::set<std::string> singleEdge = {"edge_single"};
    Topology::Corner corner(cornerId_, singleEdge, emptySurfaceIds_);

    EXPECT_EQ(corner.getConnectedEdgeIds().size(), 1);
    EXPECT_TRUE(corner.getConnectedEdgeIds().count("edge_single") > 0);
    EXPECT_TRUE(corner.getConnectedSurfaceIds().empty());
}

TEST_F(TopologyCornerTest, SingleSurfaceConnection)
{
    // Test Corner with a single surface connection
    std::set<std::string> singleSurface = {"surface_single"};
    Topology::Corner corner(cornerId_, emptyEdgeIds_, singleSurface);

    EXPECT_TRUE(corner.getConnectedEdgeIds().empty());
    EXPECT_EQ(corner.getConnectedSurfaceIds().size(), 1);
    EXPECT_TRUE(corner.getConnectedSurfaceIds().count("surface_single") > 0);
}

TEST_F(TopologyCornerTest, LargeConnectionSet)
{
    // Test Corner with a large number of connections
    std::set<std::string> manyEdges;
    std::set<std::string> manySurfaces;

    for (int i = 0; i < 100; ++i)
    {
        manyEdges.insert("edge_" + std::to_string(i));
        manySurfaces.insert("surface_" + std::to_string(i));
    }

    Topology::Corner corner(cornerId_, manyEdges, manySurfaces);

    EXPECT_EQ(corner.getConnectedEdgeIds().size(), 100);
    EXPECT_EQ(corner.getConnectedSurfaceIds().size(), 100);

    // Verify some specific entries
    EXPECT_TRUE(corner.getConnectedEdgeIds().count("edge_0") > 0);
    EXPECT_TRUE(corner.getConnectedEdgeIds().count("edge_99") > 0);
    EXPECT_TRUE(corner.getConnectedSurfaceIds().count("surface_0") > 0);
    EXPECT_TRUE(corner.getConnectedSurfaceIds().count("surface_99") > 0);
}

TEST_F(TopologyCornerTest, SpecialCharactersInIds)
{
    // Test Corner with special characters in IDs
    std::string specialId = "corner_@#$%^&*()_+{}|:<>?";
    std::set<std::string> specialEdges = {"edge-with-dashes", "edge_with_underscores", "edge.with.dots"};
    std::set<std::string> specialSurfaces = {"surface@symbol", "surface#hash", "surface$dollar"};

    Topology::Corner corner(specialId, specialEdges, specialSurfaces);

    EXPECT_EQ(corner.getId(), specialId);
    EXPECT_EQ(corner.getConnectedEdgeIds(), specialEdges);
    EXPECT_EQ(corner.getConnectedSurfaceIds(), specialSurfaces);
}