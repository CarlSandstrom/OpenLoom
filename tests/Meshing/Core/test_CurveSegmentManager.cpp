#include "Meshing/Core/3D/RCDT/CurveSegmentBuilder.h"
#include "Meshing/Core/3D/General/DiscretizationResult3D.h"
#include "Meshing/Data/CurveSegmentManager.h"

#include "Common/BoundingBox2D.h"
#include "Common/Types.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ICorner3D.h"
#include "Geometry/3D/Base/IEdge3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Topology/Corner3D.h"
#include "Topology/Edge3D.h"
#include "Topology/SeamCollection.h"
#include "Topology/Surface3D.h"
#include "Topology/Topology3D.h"

#include <array>
#include <cmath>
#include <gtest/gtest.h>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

using namespace Meshing;

namespace
{

// ============================================================================
// MockCircularArcEdge
//
// A circular arc of radius R_ in the XY plane at height z_.
//   getPoint(t)  = (R·cos(t), R·sin(t), z)
//   parameter range: [tStart_, tEnd_]
//
// Circles are uniformly parameterised (arc length ∝ parameter), so
// getParameterAtArcLengthFraction just linearly interpolates.
// ============================================================================

class MockCircularArcEdge : public Geometry3D::IEdge3D
{
public:
    MockCircularArcEdge(std::string edgeId, double radius, double z, double tStart, double tEnd) :
        edgeId_(std::move(edgeId)),
        radius_(radius),
        z_(z),
        tStart_(tStart),
        tEnd_(tEnd)
    {
    }

    Point3D getPoint(double t) const override
    {
        return Point3D(radius_ * std::cos(t), radius_ * std::sin(t), z_);
    }

    Vector3D getTangent(double t) const override
    {
        return {-radius_ * std::sin(t), radius_ * std::cos(t), 0.0};
    }

    Point3D getStartPoint() const override { return getPoint(tStart_); }
    Point3D getEndPoint() const override { return getPoint(tEnd_); }

    std::pair<double, double> getParameterBounds() const override { return {tStart_, tEnd_}; }

    double getLength() const override { return radius_ * std::abs(tEnd_ - tStart_); }

    double getParameterAtArcLengthFraction(double tStart, double tEnd, double fraction) const override
    {
        return tStart + fraction * (tEnd - tStart);
    }

    double getCurvature(double /*t*/) const override { return 1.0 / radius_; }

    std::string getId() const override { return edgeId_; }

private:
    std::string edgeId_;
    double radius_;
    double z_;
    double tStart_;
    double tEnd_;
};

// ============================================================================
// MockNonUniformEdge
//
// A straight line along the X axis with non-uniform parameterisation:
//   getPoint(t) = (t², 0, 0),  t ∈ [0, 1]
//
// The arc-length from 0 to t is ∫₀ᵗ 2s ds = t², so:
//   getParameterAtArcLengthFraction(tStart, tEnd, 0.5)
//     = sqrt( tStart² + 0.5·(tEnd² - tStart²) )
//
// For [0, 1]: parameter midpoint = 0.5 → point (0.25, 0, 0)
//             arc-length midpoint = 1/√2 ≈ 0.707 → point (0.5, 0, 0)
// ============================================================================

class MockNonUniformEdge : public Geometry3D::IEdge3D
{
public:
    Point3D getPoint(double t) const override { return Point3D(t * t, 0.0, 0.0); }

    Vector3D getTangent(double t) const override { return {2.0 * t, 0.0, 0.0}; }

    Point3D getStartPoint() const override { return getPoint(0.0); }
    Point3D getEndPoint() const override { return getPoint(1.0); }

    std::pair<double, double> getParameterBounds() const override { return {0.0, 1.0}; }

    double getLength() const override { return 1.0; }

    double getParameterAtArcLengthFraction(double tStart, double tEnd, double fraction) const override
    {
        return std::sqrt(tStart * tStart + fraction * (tEnd * tEnd - tStart * tStart));
    }

    double getCurvature(double /*t*/) const override { return 0.0; }

    std::string getId() const override { return "non_uniform_edge"; }
};

// ============================================================================
// Helpers
// ============================================================================

Geometry3D::GeometryCollection3D makeGeometry(
    std::unordered_map<std::string, std::unique_ptr<Geometry3D::IEdge3D>> edges)
{
    std::unordered_map<std::string, std::unique_ptr<Geometry3D::ISurface3D>> surfaces;
    std::unordered_map<std::string, std::unique_ptr<Geometry3D::ICorner3D>> corners;
    return Geometry3D::GeometryCollection3D(std::move(surfaces), std::move(edges),
                                            std::move(corners));
}

// Build a minimal Topology3D with the given edges.
// edgeDefs: list of (id, startCornerId, endCornerId)
// seamPairs: list of (originalEdgeId, twinEdgeId)
Topology3D::Topology3D makeTopology(
    const std::vector<std::string>& cornerIds,
    const std::vector<std::tuple<std::string, std::string, std::string>>& edgeDefs,
    const std::vector<std::pair<std::string, std::string>>& seamPairs = {})
{
    std::unordered_map<std::string, Topology3D::Corner3D> corners;
    for (const auto& cornerId : cornerIds)
        corners.emplace(cornerId, Topology3D::Corner3D(cornerId, {}, {}));

    std::unordered_map<std::string, Topology3D::Edge3D> edges;
    for (const auto& [edgeId, startId, endId] : edgeDefs)
        edges.emplace(edgeId, Topology3D::Edge3D(edgeId, startId, endId, {}));

    std::unordered_map<std::string, Topology3D::Surface3D> surfaces;

    Topology3D::SeamCollection seams;
    for (const auto& [originalId, twinId] : seamPairs)
        seams.addPair(originalId, twinId);

    return Topology3D::Topology3D(surfaces, edges, corners, seams);
}

// build only reads the two edge-sampling fields; the rest of a real
// discretization result is irrelevant to it.
DiscretizationResult3D makeDiscretization(std::map<std::string, std::vector<size_t>> edgeIdToPointIndicesMap,
                                          std::vector<std::vector<double>> edgeParameters)
{
    DiscretizationResult3D discretization;
    discretization.edgeIdToPointIndicesMap = std::move(edgeIdToPointIndicesMap);
    discretization.edgeParameters = std::move(edgeParameters);
    return discretization;
}

} // namespace

// ============================================================================
// build: cylinder-like topology (2 circular edges + seam + seam twin)
// ============================================================================

TEST(CurveSegmentManagerTest, BuildFrom_CylinderLikeTopology_SkipsSeamTwin)
{
    // Corners: c_top (point 0), c_bot (point 1)
    // Edges:
    //   "top_circle"  : c_top → c_top (full circle, self-loop)
    //   "bot_circle"  : c_bot → c_bot (full circle, self-loop)
    //   "seam"        : c_top → c_bot
    //   "seam_twin"   : c_top → c_bot (seam twin — must be skipped)

    auto topology = makeTopology(
        {"c_top", "c_bot"},
        {{"top_circle", "c_top", "c_top"},
         {"bot_circle", "c_bot", "c_bot"},
         {"seam", "c_top", "c_bot"},
         {"seam_twin", "c_top", "c_bot"}},
        {{"seam", "seam_twin"}});

    std::unordered_map<std::string, std::unique_ptr<Geometry3D::IEdge3D>> edges;
    edges["top_circle"] =
        std::make_unique<MockCircularArcEdge>("top_circle", 1.0, 1.0, 0.0, 2.0 * M_PI);
    edges["bot_circle"] =
        std::make_unique<MockCircularArcEdge>("bot_circle", 1.0, 0.0, 0.0, 2.0 * M_PI);
    edges["seam"] = std::make_unique<MockCircularArcEdge>("seam", 1.0, 0.0, 0.0, 1.0);
    edges["seam_twin"] = std::make_unique<MockCircularArcEdge>("seam_twin", 1.0, 0.0, 0.0, 1.0);
    auto geometry = makeGeometry(std::move(edges));

    // Corner-only sequences: no interior nodes.
    // Point 0 = c_top, point 1 = c_bot.
    const auto discretization = makeDiscretization({{"top_circle", {0, 0}},
                                                    {"bot_circle", {1, 1}},
                                                    {"seam",       {0, 1}}},
                                                   {{}, {}});
    const std::map<size_t, size_t> pointIndexToNodeIdMap = {{0, 0}, {1, 1}};

    auto manager =
        CurveSegmentBuilder::build(topology, geometry, discretization, pointIndexToNodeIdMap);

    // seam_twin must be skipped → 3 segments only
    EXPECT_EQ(manager.size(), 3u);

    for (const auto& [segmentId, segment] : manager.getAllSegments())
        EXPECT_NE(segment.edgeId, "seam_twin");
}

// ============================================================================
// build: parameter bounds are recorded from the edge geometry
// ============================================================================

TEST(CurveSegmentManagerTest, BuildFrom_ParameterBoundsMatchEdge)
{
    auto topology = makeTopology({"c0", "c1"}, {{"edge_a", "c0", "c1"}});

    std::unordered_map<std::string, std::unique_ptr<Geometry3D::IEdge3D>> edges;
    edges["edge_a"] = std::make_unique<MockCircularArcEdge>("edge_a", 1.0, 0.0, 0.5, 2.0);
    auto geometry = makeGeometry(std::move(edges));

    // Point 0 = c0, point 1 = c1; no interior nodes.
    const auto discretization = makeDiscretization({{"edge_a", {0, 1}}}, {{}, {}});
    const std::map<size_t, size_t> pointIndexToNodeIdMap = {{0, 10}, {1, 11}};

    auto manager =
        CurveSegmentBuilder::build(topology, geometry, discretization, pointIndexToNodeIdMap);

    ASSERT_EQ(manager.size(), 1u);
    const auto& segment = manager.getAllSegments().begin()->second;
    EXPECT_DOUBLE_EQ(segment.tStart, 0.5);
    EXPECT_DOUBLE_EQ(segment.tEnd, 2.0);
    EXPECT_EQ(segment.nodeId1, 10u);
    EXPECT_EQ(segment.nodeId2, 11u);
}

// ============================================================================
// build: edge with intermediate nodes produces one segment per gap
// ============================================================================

TEST(CurveSegmentManagerTest, BuildFrom_EdgeWithIntermediateNodes_CreatesSubSegments)
{
    // edge_b: c0 → c1, parameter range [0, π].
    // Two interior nodes at π/3 and 2π/3 → 4 points total → 3 segments.
    auto topology = makeTopology({"c0", "c1"}, {{"edge_b", "c0", "c1"}});

    std::unordered_map<std::string, std::unique_ptr<Geometry3D::IEdge3D>> edges;
    edges["edge_b"] = std::make_unique<MockCircularArcEdge>("edge_b", 1.0, 0.0, 0.0, M_PI);
    auto geometry = makeGeometry(std::move(edges));

    // Points: 0=c0 corner, 1=interior@π/3, 2=interior@2π/3, 3=c1 corner
    const auto discretization = makeDiscretization({{"edge_b", {0, 1, 2, 3}}},
                                                   {{}, {M_PI / 3.0}, {2.0 * M_PI / 3.0}, {}});
    const std::map<size_t, size_t> pointIndexToNodeIdMap = {
        {0, 10}, {1, 11}, {2, 12}, {3, 13}};

    auto manager =
        CurveSegmentBuilder::build(topology, geometry, discretization, pointIndexToNodeIdMap);

    ASSERT_EQ(manager.size(), 3u);

    const auto segments = manager.getSegmentsForEdge("edge_b");
    ASSERT_EQ(segments.size(), 3u);

    EXPECT_EQ(segments[0].nodeId1, 10u);
    EXPECT_EQ(segments[0].nodeId2, 11u);
    EXPECT_DOUBLE_EQ(segments[0].tStart, 0.0);
    EXPECT_DOUBLE_EQ(segments[0].tEnd, M_PI / 3.0);

    EXPECT_EQ(segments[1].nodeId1, 11u);
    EXPECT_EQ(segments[1].nodeId2, 12u);
    EXPECT_DOUBLE_EQ(segments[1].tStart, M_PI / 3.0);
    EXPECT_DOUBLE_EQ(segments[1].tEnd, 2.0 * M_PI / 3.0);

    EXPECT_EQ(segments[2].nodeId1, 12u);
    EXPECT_EQ(segments[2].nodeId2, 13u);
    EXPECT_DOUBLE_EQ(segments[2].tStart, 2.0 * M_PI / 3.0);
    EXPECT_DOUBLE_EQ(segments[2].tEnd, M_PI);
}

// ============================================================================
// splitAt: child segments together span the original parameter range
// ============================================================================

TEST(CurveSegmentManagerTest, SplitAt_ChildSegmentsSpanOriginalRange)
{
    CurveSegment segment;
    segment.nodeId1 = 0;
    segment.nodeId2 = 1;
    segment.edgeId = "arc";
    segment.tStart = 0.0;
    segment.tEnd = M_PI;

    CurveSegmentManager manager;
    const size_t segmentId = manager.addSegment(segment);

    // Arc-length midpoint of [0, π] on a uniform circle is π/2
    const double tMid = M_PI / 2.0;
    const auto [child1Id, child2Id] = manager.splitAt(segmentId, /*newNodeId=*/2, tMid);

    const auto& child1 = manager.getSegment(child1Id);
    const auto& child2 = manager.getSegment(child2Id);

    EXPECT_DOUBLE_EQ(child1.tStart, 0.0);
    EXPECT_DOUBLE_EQ(child1.tEnd, tMid);
    EXPECT_DOUBLE_EQ(child2.tStart, tMid);
    EXPECT_DOUBLE_EQ(child2.tEnd, M_PI);

    // Original segment must be gone
    EXPECT_EQ(manager.size(), 2u);
}

// ============================================================================
// getOrderedNodeIdsForEdge: chain follows curve order after splits
// ============================================================================

TEST(CurveSegmentManagerTest, GetOrderedNodeIdsForEdge_AfterSplits_ReturnsChainInCurveOrder)
{
    CurveSegment segment;
    segment.nodeId1 = 0;
    segment.nodeId2 = 1;
    segment.edgeId = "arc";
    segment.tStart = 0.0;
    segment.tEnd = 1.0;

    CurveSegment otherEdgeSegment;
    otherEdgeSegment.nodeId1 = 1;
    otherEdgeSegment.nodeId2 = 5;
    otherEdgeSegment.edgeId = "other";
    otherEdgeSegment.tStart = 0.0;
    otherEdgeSegment.tEnd = 1.0;

    CurveSegmentManager manager;
    const size_t segmentId = manager.addSegment(segment);
    manager.addSegment(otherEdgeSegment);

    // Split the right half after the left, so segment IDs are not in curve order
    const auto [leftId, rightId] = manager.splitAt(segmentId, /*newNodeId=*/2, 0.5);
    manager.splitAt(rightId, /*newNodeId=*/3, 0.75);
    manager.splitAt(leftId, /*newNodeId=*/4, 0.25);

    EXPECT_EQ(manager.getOrderedNodeIdsForEdge("arc"), (std::vector<size_t>{0, 4, 2, 3, 1}));
    EXPECT_TRUE(manager.getOrderedNodeIdsForEdge("missing").empty());
}

// ============================================================================
// seam handling: seam twin not registered; splitting seam produces no conflict
// ============================================================================

TEST(CurveSegmentManagerTest, SeamHandling_SeamTwinNotRegistered_SplitProducesNoConflict)
{
    auto topology = makeTopology({"c_top", "c_bot"},
                                 {{"seam", "c_top", "c_bot"}, {"seam_twin", "c_top", "c_bot"}},
                                 {{"seam", "seam_twin"}});

    std::unordered_map<std::string, std::unique_ptr<Geometry3D::IEdge3D>> edges;
    edges["seam"] = std::make_unique<MockCircularArcEdge>("seam", 1.0, 0.0, 0.0, 1.0);
    edges["seam_twin"] = std::make_unique<MockCircularArcEdge>("seam_twin", 1.0, 0.0, 0.0, 1.0);
    auto geometry = makeGeometry(std::move(edges));

    // Point 0 = c_top, point 1 = c_bot; no interior nodes.
    const auto discretization = makeDiscretization({{"seam", {0, 1}}}, {{}, {}});
    const std::map<size_t, size_t> pointIndexToNodeIdMap = {{0, 0}, {1, 1}};

    auto manager =
        CurveSegmentBuilder::build(topology, geometry, discretization, pointIndexToNodeIdMap);

    // Only one segment (seam), not two
    ASSERT_EQ(manager.size(), 1u);

    const size_t segmentId = manager.getAllSegments().begin()->first;
    const auto [child1Id, child2Id] = manager.splitAt(segmentId, /*newNodeId=*/2, 0.5);

    // Two children, original gone
    EXPECT_EQ(manager.size(), 2u);

    const auto& child1 = manager.getSegment(child1Id);
    const auto& child2 = manager.getSegment(child2Id);

    // Children share the new node and each retain one original endpoint
    EXPECT_EQ(child1.nodeId2, 2u);
    EXPECT_EQ(child2.nodeId1, 2u);
    EXPECT_NE(child1.nodeId1, child2.nodeId2);
}
