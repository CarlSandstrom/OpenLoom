#include <gtest/gtest.h>

#include "Meshing/Core/3D/General/EdgeTwinTable.h"
#include "Meshing/Core/3D/General/TwinTableGenerator.h"
#include "Topology/Corner3D.h"
#include "Topology/Edge3D.h"
#include "Topology/Surface3D.h"
#include "Topology/Topology3D.h"

#include <algorithm>
#include <unordered_map>

using namespace Meshing;

// ============================================================================
// Test topology: two triangular faces sharing one interior edge
//
//   C1 ──E12── C2 ──E24── C4
//    \         / \         /
//    E13     E23  E23    E34
//      \     /     \     /
//       C3            C3
//
//  S1 = triangle(C1, C2, C3), boundary edges: E12, E13; shared edge: E23
//  S2 = triangle(C2, C4, C3), boundary edges: E24, E34; shared edge: E23
//
// Edge adjacency:
//   E12: [S1]        — boundary
//   E13: [S1]        — boundary
//   E23: [S1, S2]    — SHARED (interior)
//   E24: [S2]        — boundary
//   E34: [S2]        — boundary
// ============================================================================

namespace
{

Topology3D::Topology3D makeTriangleStripTopology()
{
    namespace T = Topology3D;

    std::unordered_map<std::string, T::Corner3D> corners;
    corners.emplace("C1", T::Corner3D("C1", {"E12", "E13"}, {"S1"}));
    corners.emplace("C2", T::Corner3D("C2", {"E12", "E23", "E24"}, {"S1", "S2"}));
    corners.emplace("C3", T::Corner3D("C3", {"E13", "E23", "E34"}, {"S1", "S2"}));
    corners.emplace("C4", T::Corner3D("C4", {"E24", "E34"}, {"S2"}));

    std::unordered_map<std::string, T::Edge3D> edges;
    edges.emplace("E12", T::Edge3D("E12", "C1", "C2", {"S1"}));
    edges.emplace("E13", T::Edge3D("E13", "C1", "C3", {"S1"}));
    edges.emplace("E23", T::Edge3D("E23", "C2", "C3", {"S1", "S2"}));
    edges.emplace("E24", T::Edge3D("E24", "C2", "C4", {"S2"}));
    edges.emplace("E34", T::Edge3D("E34", "C3", "C4", {"S2"}));

    std::unordered_map<std::string, T::Surface3D> surfaces;
    surfaces.emplace("S1", T::Surface3D("S1", {"E12", "E23", "E13"}, {"C1", "C2", "C3"}, {"S2"}));
    surfaces.emplace("S2", T::Surface3D("S2", {"E23", "E24", "E34"}, {"C2", "C4", "C3"}, {"S1"}));

    return T::Topology3D(surfaces, edges, corners);
}

Topology3D::Topology3D makeNonManifoldTopology()
{
    // Same triangle strip, plus a third surface S3 also sharing E23.
    namespace T = Topology3D;

    std::unordered_map<std::string, T::Corner3D> corners;
    corners.emplace("C1", T::Corner3D("C1", {"E12", "E13"}, {"S1"}));
    corners.emplace("C2", T::Corner3D("C2", {"E12", "E23", "E24"}, {"S1", "S2"}));
    corners.emplace("C3", T::Corner3D("C3", {"E13", "E23", "E34"}, {"S1", "S2"}));
    corners.emplace("C4", T::Corner3D("C4", {"E24", "E34"}, {"S2"}));
    corners.emplace("C5", T::Corner3D("C5", {"E25", "E35"}, {"S3"}));

    std::unordered_map<std::string, T::Edge3D> edges;
    edges.emplace("E12", T::Edge3D("E12", "C1", "C2", {"S1"}));
    edges.emplace("E13", T::Edge3D("E13", "C1", "C3", {"S1"}));
    edges.emplace("E23", T::Edge3D("E23", "C2", "C3", {"S1", "S2", "S3"}));
    edges.emplace("E24", T::Edge3D("E24", "C2", "C4", {"S2"}));
    edges.emplace("E34", T::Edge3D("E34", "C3", "C4", {"S2"}));
    edges.emplace("E25", T::Edge3D("E25", "C2", "C5", {"S3"}));
    edges.emplace("E35", T::Edge3D("E35", "C3", "C5", {"S3"}));

    std::unordered_map<std::string, T::Surface3D> surfaces;
    surfaces.emplace("S1", T::Surface3D("S1", {"E12", "E23", "E13"}, {"C1", "C2", "C3"}, {"S2", "S3"}));
    surfaces.emplace("S2", T::Surface3D("S2", {"E23", "E24", "E34"}, {"C2", "C4", "C3"}, {"S1", "S3"}));
    surfaces.emplace("S3", T::Surface3D("S3", {"E23", "E25", "E35"}, {"C2", "C3", "C5"}, {"S1", "S2"}));

    return T::Topology3D(surfaces, edges, corners);
}

} // namespace

// ============================================================================
// Tests
// ============================================================================

TEST(TwinTableGeneratorTest, EmptyTopologyReturnsEmptyTable)
{
    Topology3D::Topology3D topology({}, {}, {});
    auto table = TwinTableGenerator::generate(topology);
    EXPECT_TRUE(table.empty());
}

TEST(TwinTableGeneratorTest, SharedEdgeAppearsInTable)
{
    auto topology = makeTriangleStripTopology();
    auto table = TwinTableGenerator::generate(topology);

    EXPECT_TRUE(table.contains("E23"));
}

TEST(TwinTableGeneratorTest, BoundaryEdgesOmittedFromTable)
{
    auto topology = makeTriangleStripTopology();
    auto table = TwinTableGenerator::generate(topology);

    EXPECT_FALSE(table.contains("E12"));
    EXPECT_FALSE(table.contains("E13"));
    EXPECT_FALSE(table.contains("E24"));
    EXPECT_FALSE(table.contains("E34"));
}

TEST(TwinTableGeneratorTest, SharedEdgeHasExactlyTwoEntries)
{
    auto topology = makeTriangleStripTopology();
    auto table = TwinTableGenerator::generate(topology);

    ASSERT_TRUE(table.contains("E23"));
    EXPECT_EQ(table.at("E23").size(), 2u);
}

TEST(TwinTableGeneratorTest, SharedEdgeContainsBothSurfaces)
{
    auto topology = makeTriangleStripTopology();
    auto table = TwinTableGenerator::generate(topology);

    ASSERT_TRUE(table.contains("E23"));
    const auto& entries = table.at("E23");

    auto hasSurface = [&](const std::string& id)
    {
        return std::any_of(entries.begin(), entries.end(),
                           [&](const EdgeTwinEntry& e) { return e.surfaceId == id; });
    };

    EXPECT_TRUE(hasSurface("S1"));
    EXPECT_TRUE(hasSurface("S2"));
}

TEST(TwinTableGeneratorTest, FirstSurfaceIsSameSecondIsReversed)
{
    auto topology = makeTriangleStripTopology();
    auto table = TwinTableGenerator::generate(topology);

    ASSERT_TRUE(table.contains("E23"));
    const auto& entries = table.at("E23");

    ASSERT_EQ(entries.size(), 2u);
    EXPECT_EQ(entries[0].orientation, TwinOrientation::Same);
    EXPECT_EQ(entries[1].orientation, TwinOrientation::Reversed);
}

TEST(TwinTableGeneratorTest, NonManifoldEdgeHasThreeEntries)
{
    auto topology = makeNonManifoldTopology();
    auto table = TwinTableGenerator::generate(topology);

    ASSERT_TRUE(table.contains("E23"));
    EXPECT_EQ(table.at("E23").size(), 3u);
}

TEST(TwinTableGeneratorTest, NonManifoldEdgeFirstIsSameRestAreReversed)
{
    auto topology = makeNonManifoldTopology();
    auto table = TwinTableGenerator::generate(topology);

    ASSERT_TRUE(table.contains("E23"));
    const auto& entries = table.at("E23");

    ASSERT_EQ(entries.size(), 3u);
    EXPECT_EQ(entries[0].orientation, TwinOrientation::Same);
    EXPECT_EQ(entries[1].orientation, TwinOrientation::Reversed);
    EXPECT_EQ(entries[2].orientation, TwinOrientation::Reversed);
}

TEST(TwinTableGeneratorTest, OnlySharedEdgesProduceEntries)
{
    auto topology = makeTriangleStripTopology();
    auto table = TwinTableGenerator::generate(topology);

    // 5 edges total, only E23 is shared
    EXPECT_EQ(table.size(), 1u);
}
