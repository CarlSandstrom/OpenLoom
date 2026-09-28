#include <gtest/gtest.h>

#include "Common/TwinManager.h"

// ============================================================================
// Unit tests for TwinManager (surface-aware API)
//
// Segment pair notation: (surface,n1,n2)↔(twinSurf,m1,m2).
// NO_SURFACE is used for 2D (context-free) registrations.
// ============================================================================

TEST(TwinManagerTest, RegisterAndLookup)
{
    TwinManager twinManager;
    twinManager.registerTwin(TwinManager::NO_SURFACE, 1, 2, TwinManager::NO_SURFACE, 3, 4);

    auto twin = twinManager.getTwin(TwinManager::NO_SURFACE, 1, 2);
    ASSERT_TRUE(twin.has_value());
    EXPECT_EQ(std::get<1>(*twin), 3u);
    EXPECT_EQ(std::get<2>(*twin), 4u);
}

TEST(TwinManagerTest, ReverseDirectionLookup)
{
    TwinManager twinManager;
    twinManager.registerTwin(TwinManager::NO_SURFACE, 1, 2, TwinManager::NO_SURFACE, 3, 4);

    auto twin = twinManager.getTwin(TwinManager::NO_SURFACE, 2, 1);
    ASSERT_TRUE(twin.has_value());
    EXPECT_EQ(std::get<1>(*twin), 4u);
    EXPECT_EQ(std::get<2>(*twin), 3u);
}

TEST(TwinManagerTest, SymmetricLookup)
{
    TwinManager twinManager;
    twinManager.registerTwin(TwinManager::NO_SURFACE, 1, 2, TwinManager::NO_SURFACE, 3, 4);

    auto twin = twinManager.getTwin(TwinManager::NO_SURFACE, 3, 4);
    ASSERT_TRUE(twin.has_value());
    EXPECT_EQ(std::get<1>(*twin), 1u);
    EXPECT_EQ(std::get<2>(*twin), 2u);
}

TEST(TwinManagerTest, SymmetricReverseLookup)
{
    TwinManager twinManager;
    twinManager.registerTwin(TwinManager::NO_SURFACE, 1, 2, TwinManager::NO_SURFACE, 3, 4);

    auto twin = twinManager.getTwin(TwinManager::NO_SURFACE, 4, 3);
    ASSERT_TRUE(twin.has_value());
    EXPECT_EQ(std::get<1>(*twin), 2u);
    EXPECT_EQ(std::get<2>(*twin), 1u);
}

TEST(TwinManagerTest, HasTwinTrue)
{
    TwinManager twinManager;
    twinManager.registerTwin(TwinManager::NO_SURFACE, 1, 2, TwinManager::NO_SURFACE, 3, 4);

    EXPECT_TRUE(twinManager.hasTwin(TwinManager::NO_SURFACE, 1, 2));
    EXPECT_TRUE(twinManager.hasTwin(TwinManager::NO_SURFACE, 2, 1));
    EXPECT_TRUE(twinManager.hasTwin(TwinManager::NO_SURFACE, 3, 4));
    EXPECT_TRUE(twinManager.hasTwin(TwinManager::NO_SURFACE, 4, 3));
}

TEST(TwinManagerTest, HasTwinFalse)
{
    TwinManager twinManager;
    twinManager.registerTwin(TwinManager::NO_SURFACE, 1, 2, TwinManager::NO_SURFACE, 3, 4);

    EXPECT_FALSE(twinManager.hasTwin(TwinManager::NO_SURFACE, 5, 6));
    EXPECT_FALSE(twinManager.hasTwin(TwinManager::NO_SURFACE, 1, 3));
}

TEST(TwinManagerTest, NoTwinReturnsNullopt)
{
    TwinManager twinManager;
    EXPECT_FALSE(twinManager.getTwin(TwinManager::NO_SURFACE, 5, 6).has_value());
}

TEST(TwinManagerTest, RecordSplit)
{
    TwinManager twinManager;
    twinManager.registerTwin(TwinManager::NO_SURFACE, 1, 2, TwinManager::NO_SURFACE, 3, 4);

    twinManager.recordSplit(TwinManager::NO_SURFACE, 1, 2, 9, TwinManager::NO_SURFACE, 3, 4, 99);

    EXPECT_FALSE(twinManager.hasTwin(TwinManager::NO_SURFACE, 1, 2));
    EXPECT_FALSE(twinManager.hasTwin(TwinManager::NO_SURFACE, 3, 4));

    auto twin1 = twinManager.getTwin(TwinManager::NO_SURFACE, 1, 9);
    ASSERT_TRUE(twin1.has_value());
    EXPECT_EQ(std::get<1>(*twin1), 3u);
    EXPECT_EQ(std::get<2>(*twin1), 99u);

    auto twin2 = twinManager.getTwin(TwinManager::NO_SURFACE, 9, 2);
    ASSERT_TRUE(twin2.has_value());
    EXPECT_EQ(std::get<1>(*twin2), 99u);
    EXPECT_EQ(std::get<2>(*twin2), 4u);

    auto twin1Reversed = twinManager.getTwin(TwinManager::NO_SURFACE, 9, 1);
    ASSERT_TRUE(twin1Reversed.has_value());
    EXPECT_EQ(std::get<1>(*twin1Reversed), 99u);
    EXPECT_EQ(std::get<2>(*twin1Reversed), 3u);

    auto twin1FromOtherSide = twinManager.getTwin(TwinManager::NO_SURFACE, 3, 99);
    ASSERT_TRUE(twin1FromOtherSide.has_value());
    EXPECT_EQ(std::get<1>(*twin1FromOtherSide), 1u);
    EXPECT_EQ(std::get<2>(*twin1FromOtherSide), 9u);
}

TEST(TwinManagerTest, IndependentPairs)
{
    TwinManager twinManager;
    twinManager.registerTwin(TwinManager::NO_SURFACE, 1, 2, TwinManager::NO_SURFACE, 3, 4);
    twinManager.registerTwin(TwinManager::NO_SURFACE, 10, 20, TwinManager::NO_SURFACE, 30, 40);

    auto twin1 = twinManager.getTwin(TwinManager::NO_SURFACE, 1, 2);
    ASSERT_TRUE(twin1.has_value());
    EXPECT_EQ(std::get<1>(*twin1), 3u);

    auto twin2 = twinManager.getTwin(TwinManager::NO_SURFACE, 10, 20);
    ASSERT_TRUE(twin2.has_value());
    EXPECT_EQ(std::get<1>(*twin2), 30u);
    EXPECT_EQ(std::get<2>(*twin2), 40u);

    EXPECT_FALSE(twinManager.hasTwin(TwinManager::NO_SURFACE, 1, 10));
    EXPECT_FALSE(twinManager.hasTwin(TwinManager::NO_SURFACE, 3, 30));
}

TEST(TwinManagerTest, CrossSurfaceTwins)
{
    TwinManager twinManager;
    twinManager.registerTwin("S1", 1, 2, "S2", 3, 4);

    auto twin = twinManager.getTwin("S1", 1, 2);
    ASSERT_TRUE(twin.has_value());
    EXPECT_EQ(std::get<0>(*twin), "S2");
    EXPECT_EQ(std::get<1>(*twin), 3u);
    EXPECT_EQ(std::get<2>(*twin), 4u);

    auto twinReversed = twinManager.getTwin("S2", 3, 4);
    ASSERT_TRUE(twinReversed.has_value());
    EXPECT_EQ(std::get<0>(*twinReversed), "S1");
    EXPECT_EQ(std::get<1>(*twinReversed), 1u);
    EXPECT_EQ(std::get<2>(*twinReversed), 2u);

    // Different surface — no twin
    EXPECT_FALSE(twinManager.getTwin("S3", 1, 2).has_value());
    EXPECT_FALSE(twinManager.getTwin(TwinManager::NO_SURFACE, 1, 2).has_value());
}
