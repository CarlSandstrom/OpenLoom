#include <gtest/gtest.h>

#include "Common/TwinManager.h"

// ============================================================================
// Unit tests for TwinManager
//
// Segment pair notation: (n1,n2)↔(m1,m2) means n1↔m1 and n2↔m2.
// ============================================================================

TEST(TwinManagerTest, RegisterAndLookup)
{
    TwinManager tm;
    tm.registerTwin(1, 2, 3, 4);

    auto twin = tm.getTwin(1, 2);
    ASSERT_TRUE(twin.has_value());
    EXPECT_EQ(twin->first, 3u);
    EXPECT_EQ(twin->second, 4u);
}

TEST(TwinManagerTest, ReverseDirectionLookup)
{
    TwinManager tm;
    tm.registerTwin(1, 2, 3, 4);

    // Segment stored as (2,1) in a ConstrainedSegment2D — should still find twin
    auto twin = tm.getTwin(2, 1);
    ASSERT_TRUE(twin.has_value());
    EXPECT_EQ(twin->first, 4u);
    EXPECT_EQ(twin->second, 3u);
}

TEST(TwinManagerTest, SymmetricLookup)
{
    TwinManager tm;
    tm.registerTwin(1, 2, 3, 4);

    // Looking up from the twin side
    auto twin = tm.getTwin(3, 4);
    ASSERT_TRUE(twin.has_value());
    EXPECT_EQ(twin->first, 1u);
    EXPECT_EQ(twin->second, 2u);
}

TEST(TwinManagerTest, SymmetricReverseLookup)
{
    TwinManager tm;
    tm.registerTwin(1, 2, 3, 4);

    auto twin = tm.getTwin(4, 3);
    ASSERT_TRUE(twin.has_value());
    EXPECT_EQ(twin->first, 2u);
    EXPECT_EQ(twin->second, 1u);
}

TEST(TwinManagerTest, HasTwinTrue)
{
    TwinManager tm;
    tm.registerTwin(1, 2, 3, 4);

    EXPECT_TRUE(tm.hasTwin(1, 2));
    EXPECT_TRUE(tm.hasTwin(2, 1));
    EXPECT_TRUE(tm.hasTwin(3, 4));
    EXPECT_TRUE(tm.hasTwin(4, 3));
}

TEST(TwinManagerTest, HasTwinFalse)
{
    TwinManager tm;
    tm.registerTwin(1, 2, 3, 4);

    EXPECT_FALSE(tm.hasTwin(5, 6));
    EXPECT_FALSE(tm.hasTwin(1, 3)); // different pair
}

TEST(TwinManagerTest, NoTwinReturnsNullopt)
{
    TwinManager tm;
    EXPECT_FALSE(tm.getTwin(5, 6).has_value());
}

TEST(TwinManagerTest, RecordSplit)
{
    TwinManager tm;
    tm.registerTwin(1, 2, 3, 4);

    // Split: {1,2} at mid=9, twin {3,4} at twinMid=99
    tm.recordSplit(1, 2, 9, 3, 4, 99);

    // Old pair gone
    EXPECT_FALSE(tm.hasTwin(1, 2));
    EXPECT_FALSE(tm.hasTwin(3, 4));

    // New sub-segment pairs present with correct endpoint correspondence
    auto tw1 = tm.getTwin(1, 9);
    ASSERT_TRUE(tw1.has_value());
    EXPECT_EQ(tw1->first, 3u);
    EXPECT_EQ(tw1->second, 99u);

    auto tw2 = tm.getTwin(9, 2);
    ASSERT_TRUE(tw2.has_value());
    EXPECT_EQ(tw2->first, 99u);
    EXPECT_EQ(tw2->second, 4u);

    // Reverse directions
    auto tw1r = tm.getTwin(9, 1);
    ASSERT_TRUE(tw1r.has_value());
    EXPECT_EQ(tw1r->first, 99u);
    EXPECT_EQ(tw1r->second, 3u);

    // Symmetric
    auto tw1s = tm.getTwin(3, 99);
    ASSERT_TRUE(tw1s.has_value());
    EXPECT_EQ(tw1s->first, 1u);
    EXPECT_EQ(tw1s->second, 9u);
}

TEST(TwinManagerTest, IndependentPairs)
{
    TwinManager tm;
    tm.registerTwin(1, 2, 3, 4);
    tm.registerTwin(10, 20, 30, 40);

    // First pair unaffected
    auto tw1 = tm.getTwin(1, 2);
    ASSERT_TRUE(tw1.has_value());
    EXPECT_EQ(tw1->first, 3u);

    // Second pair works
    auto tw2 = tm.getTwin(10, 20);
    ASSERT_TRUE(tw2.has_value());
    EXPECT_EQ(tw2->first, 30u);
    EXPECT_EQ(tw2->second, 40u);

    // No cross-contamination
    EXPECT_FALSE(tm.hasTwin(1, 10));
    EXPECT_FALSE(tm.hasTwin(3, 30));
}
