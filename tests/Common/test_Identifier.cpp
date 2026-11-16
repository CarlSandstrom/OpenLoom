#include "Common/Identifier.h"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <type_traits>
#include <unordered_set>

using namespace Meshing;

namespace
{

using NodeId = Identifier<struct NodeTag>;
using ElementId = Identifier<struct ElementTag>;

static_assert(!std::is_convertible_v<NodeId, ElementId>);
static_assert(!std::is_convertible_v<size_t, NodeId>);
static_assert(!std::is_convertible_v<NodeId, size_t>);
static_assert(std::is_trivially_copyable_v<NodeId>);

} // namespace

TEST(Identifier, DefaultConstructsToZero)
{
    EXPECT_EQ(NodeId{}.value(), 0u);
}

TEST(Identifier, ValueRoundTrips)
{
    EXPECT_EQ(NodeId{42}.value(), 42u);
}

TEST(Identifier, EqualityComparesValues)
{
    EXPECT_EQ(NodeId{3}, NodeId{3});
    EXPECT_NE(NodeId{3}, NodeId{4});
}

TEST(Identifier, OrderingFollowsValues)
{
    EXPECT_LT(NodeId{1}, NodeId{2});
    EXPECT_GT(NodeId{5}, NodeId{2});
    EXPECT_LE(NodeId{2}, NodeId{2});
}

TEST(Identifier, HashIsTheIdentityHashOfTheValue)
{
    EXPECT_EQ(std::hash<NodeId>{}(NodeId{7}), std::hash<size_t>{}(7));
}

TEST(Identifier, WorksAsUnorderedContainerKey)
{
    std::unordered_set<NodeId> nodes = {NodeId{1}, NodeId{2}, NodeId{1}};
    EXPECT_EQ(nodes.size(), 2u);
}

TEST(Identifier, InvalidIsTheMaximumValue)
{
    EXPECT_EQ(ElementId::INVALID.value(), SIZE_MAX);
    EXPECT_NE(ElementId::INVALID, ElementId{0});
}
