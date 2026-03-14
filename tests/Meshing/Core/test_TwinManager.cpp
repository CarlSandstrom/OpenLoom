#include <gtest/gtest.h>

#include "Common/TwinManager.h"

// ============================================================================
// Unit tests for TwinManager (surface-aware API)
//
// Segment pair notation: (surf,n1,n2)↔(twinSurf,m1,m2).
// NO_SURFACE is used for 2D (context-free) registrations.
// ============================================================================

TEST(TwinManagerTest, RegisterAndLookup)
{
    TwinManager tm;
    tm.registerTwin(TwinManager::NO_SURFACE, 1, 2, TwinManager::NO_SURFACE, 3, 4);

    auto twin = tm.getTwin(TwinManager::NO_SURFACE, 1, 2);
    ASSERT_TRUE(twin.has_value());
    EXPECT_EQ(std::get<1>(*twin), 3u);
    EXPECT_EQ(std::get<2>(*twin), 4u);
}

TEST(TwinManagerTest, ReverseDirectionLookup)
{
    TwinManager tm;
    tm.registerTwin(TwinManager::NO_SURFACE, 1, 2, TwinManager::NO_SURFACE, 3, 4);

    auto twin = tm.getTwin(TwinManager::NO_SURFACE, 2, 1);
    ASSERT_TRUE(twin.has_value());
    EXPECT_EQ(std::get<1>(*twin), 4u);
    EXPECT_EQ(std::get<2>(*twin), 3u);
}

TEST(TwinManagerTest, SymmetricLookup)
{
    TwinManager tm;
    tm.registerTwin(TwinManager::NO_SURFACE, 1, 2, TwinManager::NO_SURFACE, 3, 4);

    auto twin = tm.getTwin(TwinManager::NO_SURFACE, 3, 4);
    ASSERT_TRUE(twin.has_value());
    EXPECT_EQ(std::get<1>(*twin), 1u);
    EXPECT_EQ(std::get<2>(*twin), 2u);
}

TEST(TwinManagerTest, SymmetricReverseLookup)
{
    TwinManager tm;
    tm.registerTwin(TwinManager::NO_SURFACE, 1, 2, TwinManager::NO_SURFACE, 3, 4);

    auto twin = tm.getTwin(TwinManager::NO_SURFACE, 4, 3);
    ASSERT_TRUE(twin.has_value());
    EXPECT_EQ(std::get<1>(*twin), 2u);
    EXPECT_EQ(std::get<2>(*twin), 1u);
}

TEST(TwinManagerTest, HasTwinTrue)
{
    TwinManager tm;
    tm.registerTwin(TwinManager::NO_SURFACE, 1, 2, TwinManager::NO_SURFACE, 3, 4);

    EXPECT_TRUE(tm.hasTwin(TwinManager::NO_SURFACE, 1, 2));
    EXPECT_TRUE(tm.hasTwin(TwinManager::NO_SURFACE, 2, 1));
    EXPECT_TRUE(tm.hasTwin(TwinManager::NO_SURFACE, 3, 4));
    EXPECT_TRUE(tm.hasTwin(TwinManager::NO_SURFACE, 4, 3));
}

TEST(TwinManagerTest, HasTwinFalse)
{
    TwinManager tm;
    tm.registerTwin(TwinManager::NO_SURFACE, 1, 2, TwinManager::NO_SURFACE, 3, 4);

    EXPECT_FALSE(tm.hasTwin(TwinManager::NO_SURFACE, 5, 6));
    EXPECT_FALSE(tm.hasTwin(TwinManager::NO_SURFACE, 1, 3));
}

TEST(TwinManagerTest, NoTwinReturnsNullopt)
{
    TwinManager tm;
    EXPECT_FALSE(tm.getTwin(TwinManager::NO_SURFACE, 5, 6).has_value());
}

TEST(TwinManagerTest, RecordSplit)
{
    TwinManager tm;
    tm.registerTwin(TwinManager::NO_SURFACE, 1, 2, TwinManager::NO_SURFACE, 3, 4);

    tm.recordSplit(TwinManager::NO_SURFACE, 1, 2, 9, TwinManager::NO_SURFACE, 3, 4, 99);

    EXPECT_FALSE(tm.hasTwin(TwinManager::NO_SURFACE, 1, 2));
    EXPECT_FALSE(tm.hasTwin(TwinManager::NO_SURFACE, 3, 4));

    auto tw1 = tm.getTwin(TwinManager::NO_SURFACE, 1, 9);
    ASSERT_TRUE(tw1.has_value());
    EXPECT_EQ(std::get<1>(*tw1), 3u);
    EXPECT_EQ(std::get<2>(*tw1), 99u);

    auto tw2 = tm.getTwin(TwinManager::NO_SURFACE, 9, 2);
    ASSERT_TRUE(tw2.has_value());
    EXPECT_EQ(std::get<1>(*tw2), 99u);
    EXPECT_EQ(std::get<2>(*tw2), 4u);

    auto tw1r = tm.getTwin(TwinManager::NO_SURFACE, 9, 1);
    ASSERT_TRUE(tw1r.has_value());
    EXPECT_EQ(std::get<1>(*tw1r), 99u);
    EXPECT_EQ(std::get<2>(*tw1r), 3u);

    auto tw1s = tm.getTwin(TwinManager::NO_SURFACE, 3, 99);
    ASSERT_TRUE(tw1s.has_value());
    EXPECT_EQ(std::get<1>(*tw1s), 1u);
    EXPECT_EQ(std::get<2>(*tw1s), 9u);
}

TEST(TwinManagerTest, IndependentPairs)
{
    TwinManager tm;
    tm.registerTwin(TwinManager::NO_SURFACE, 1, 2, TwinManager::NO_SURFACE, 3, 4);
    tm.registerTwin(TwinManager::NO_SURFACE, 10, 20, TwinManager::NO_SURFACE, 30, 40);

    auto tw1 = tm.getTwin(TwinManager::NO_SURFACE, 1, 2);
    ASSERT_TRUE(tw1.has_value());
    EXPECT_EQ(std::get<1>(*tw1), 3u);

    auto tw2 = tm.getTwin(TwinManager::NO_SURFACE, 10, 20);
    ASSERT_TRUE(tw2.has_value());
    EXPECT_EQ(std::get<1>(*tw2), 30u);
    EXPECT_EQ(std::get<2>(*tw2), 40u);

    EXPECT_FALSE(tm.hasTwin(TwinManager::NO_SURFACE, 1, 10));
    EXPECT_FALSE(tm.hasTwin(TwinManager::NO_SURFACE, 3, 30));
}

TEST(TwinManagerTest, CrossSurfaceTwins)
{
    TwinManager tm;
    tm.registerTwin("S1", 1, 2, "S2", 3, 4);

    auto twin = tm.getTwin("S1", 1, 2);
    ASSERT_TRUE(twin.has_value());
    EXPECT_EQ(std::get<0>(*twin), "S2");
    EXPECT_EQ(std::get<1>(*twin), 3u);
    EXPECT_EQ(std::get<2>(*twin), 4u);

    auto twinRev = tm.getTwin("S2", 3, 4);
    ASSERT_TRUE(twinRev.has_value());
    EXPECT_EQ(std::get<0>(*twinRev), "S1");
    EXPECT_EQ(std::get<1>(*twinRev), 1u);
    EXPECT_EQ(std::get<2>(*twinRev), 2u);

    // Different surface — no twin
    EXPECT_FALSE(tm.getTwin("S3", 1, 2).has_value());
    EXPECT_FALSE(tm.getTwin(TwinManager::NO_SURFACE, 1, 2).has_value());
}
