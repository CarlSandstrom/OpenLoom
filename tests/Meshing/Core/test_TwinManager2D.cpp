#include <gtest/gtest.h>

#include "Meshing/Core/2D/Periodic/TwinManager2D.h"

// ============================================================================
// Unit tests for TwinManager2D (surface-aware API)
//
// Segment pair notation: (surface,n1,n2)↔(twinSurf,m1,m2).
// NO_SURFACE is used for 2D (context-free) registrations.
// ============================================================================

TEST(TwinManager2DTest, RegisterAndLookup)
{
    TwinManager2D twinManager;
    twinManager.registerTwin(TwinManager2D::NO_SURFACE, Meshing::NodeId{1}, Meshing::NodeId{2}, TwinManager2D::NO_SURFACE, Meshing::NodeId{3}, Meshing::NodeId{4});

    auto twin = twinManager.getTwin(TwinManager2D::NO_SURFACE, Meshing::NodeId{1}, Meshing::NodeId{2});
    ASSERT_TRUE(twin.has_value());
    EXPECT_EQ(std::get<1>(*twin), Meshing::NodeId{3});
    EXPECT_EQ(std::get<2>(*twin), Meshing::NodeId{4});
}

TEST(TwinManager2DTest, ReverseDirectionLookup)
{
    TwinManager2D twinManager;
    twinManager.registerTwin(TwinManager2D::NO_SURFACE, Meshing::NodeId{1}, Meshing::NodeId{2}, TwinManager2D::NO_SURFACE, Meshing::NodeId{3}, Meshing::NodeId{4});

    auto twin = twinManager.getTwin(TwinManager2D::NO_SURFACE, Meshing::NodeId{2}, Meshing::NodeId{1});
    ASSERT_TRUE(twin.has_value());
    EXPECT_EQ(std::get<1>(*twin), Meshing::NodeId{4});
    EXPECT_EQ(std::get<2>(*twin), Meshing::NodeId{3});
}

TEST(TwinManager2DTest, SymmetricLookup)
{
    TwinManager2D twinManager;
    twinManager.registerTwin(TwinManager2D::NO_SURFACE, Meshing::NodeId{1}, Meshing::NodeId{2}, TwinManager2D::NO_SURFACE, Meshing::NodeId{3}, Meshing::NodeId{4});

    auto twin = twinManager.getTwin(TwinManager2D::NO_SURFACE, Meshing::NodeId{3}, Meshing::NodeId{4});
    ASSERT_TRUE(twin.has_value());
    EXPECT_EQ(std::get<1>(*twin), Meshing::NodeId{1});
    EXPECT_EQ(std::get<2>(*twin), Meshing::NodeId{2});
}

TEST(TwinManager2DTest, SymmetricReverseLookup)
{
    TwinManager2D twinManager;
    twinManager.registerTwin(TwinManager2D::NO_SURFACE, Meshing::NodeId{1}, Meshing::NodeId{2}, TwinManager2D::NO_SURFACE, Meshing::NodeId{3}, Meshing::NodeId{4});

    auto twin = twinManager.getTwin(TwinManager2D::NO_SURFACE, Meshing::NodeId{4}, Meshing::NodeId{3});
    ASSERT_TRUE(twin.has_value());
    EXPECT_EQ(std::get<1>(*twin), Meshing::NodeId{2});
    EXPECT_EQ(std::get<2>(*twin), Meshing::NodeId{1});
}

TEST(TwinManager2DTest, HasTwinTrue)
{
    TwinManager2D twinManager;
    twinManager.registerTwin(TwinManager2D::NO_SURFACE, Meshing::NodeId{1}, Meshing::NodeId{2}, TwinManager2D::NO_SURFACE, Meshing::NodeId{3}, Meshing::NodeId{4});

    EXPECT_TRUE(twinManager.hasTwin(TwinManager2D::NO_SURFACE, Meshing::NodeId{1}, Meshing::NodeId{2}));
    EXPECT_TRUE(twinManager.hasTwin(TwinManager2D::NO_SURFACE, Meshing::NodeId{2}, Meshing::NodeId{1}));
    EXPECT_TRUE(twinManager.hasTwin(TwinManager2D::NO_SURFACE, Meshing::NodeId{3}, Meshing::NodeId{4}));
    EXPECT_TRUE(twinManager.hasTwin(TwinManager2D::NO_SURFACE, Meshing::NodeId{4}, Meshing::NodeId{3}));
}

TEST(TwinManager2DTest, HasTwinFalse)
{
    TwinManager2D twinManager;
    twinManager.registerTwin(TwinManager2D::NO_SURFACE, Meshing::NodeId{1}, Meshing::NodeId{2}, TwinManager2D::NO_SURFACE, Meshing::NodeId{3}, Meshing::NodeId{4});

    EXPECT_FALSE(twinManager.hasTwin(TwinManager2D::NO_SURFACE, Meshing::NodeId{5}, Meshing::NodeId{6}));
    EXPECT_FALSE(twinManager.hasTwin(TwinManager2D::NO_SURFACE, Meshing::NodeId{1}, Meshing::NodeId{3}));
}

TEST(TwinManager2DTest, NoTwinReturnsNullopt)
{
    TwinManager2D twinManager;
    EXPECT_FALSE(twinManager.getTwin(TwinManager2D::NO_SURFACE, Meshing::NodeId{5}, Meshing::NodeId{6}).has_value());
}

TEST(TwinManager2DTest, RecordSplit)
{
    TwinManager2D twinManager;
    twinManager.registerTwin(TwinManager2D::NO_SURFACE, Meshing::NodeId{1}, Meshing::NodeId{2}, TwinManager2D::NO_SURFACE, Meshing::NodeId{3}, Meshing::NodeId{4});

    twinManager.recordSplit(TwinManager2D::NO_SURFACE, Meshing::NodeId{1}, Meshing::NodeId{2}, Meshing::NodeId{9}, TwinManager2D::NO_SURFACE, Meshing::NodeId{3}, Meshing::NodeId{4}, Meshing::NodeId{99});

    EXPECT_FALSE(twinManager.hasTwin(TwinManager2D::NO_SURFACE, Meshing::NodeId{1}, Meshing::NodeId{2}));
    EXPECT_FALSE(twinManager.hasTwin(TwinManager2D::NO_SURFACE, Meshing::NodeId{3}, Meshing::NodeId{4}));

    auto twin1 = twinManager.getTwin(TwinManager2D::NO_SURFACE, Meshing::NodeId{1}, Meshing::NodeId{9});
    ASSERT_TRUE(twin1.has_value());
    EXPECT_EQ(std::get<1>(*twin1), Meshing::NodeId{3});
    EXPECT_EQ(std::get<2>(*twin1), Meshing::NodeId{99});

    auto twin2 = twinManager.getTwin(TwinManager2D::NO_SURFACE, Meshing::NodeId{9}, Meshing::NodeId{2});
    ASSERT_TRUE(twin2.has_value());
    EXPECT_EQ(std::get<1>(*twin2), Meshing::NodeId{99});
    EXPECT_EQ(std::get<2>(*twin2), Meshing::NodeId{4});

    auto twin1Reversed = twinManager.getTwin(TwinManager2D::NO_SURFACE, Meshing::NodeId{9}, Meshing::NodeId{1});
    ASSERT_TRUE(twin1Reversed.has_value());
    EXPECT_EQ(std::get<1>(*twin1Reversed), Meshing::NodeId{99});
    EXPECT_EQ(std::get<2>(*twin1Reversed), Meshing::NodeId{3});

    auto twin1FromOtherSide = twinManager.getTwin(TwinManager2D::NO_SURFACE, Meshing::NodeId{3}, Meshing::NodeId{99});
    ASSERT_TRUE(twin1FromOtherSide.has_value());
    EXPECT_EQ(std::get<1>(*twin1FromOtherSide), Meshing::NodeId{1});
    EXPECT_EQ(std::get<2>(*twin1FromOtherSide), Meshing::NodeId{9});
}

TEST(TwinManager2DTest, IndependentPairs)
{
    TwinManager2D twinManager;
    twinManager.registerTwin(TwinManager2D::NO_SURFACE, Meshing::NodeId{1}, Meshing::NodeId{2}, TwinManager2D::NO_SURFACE, Meshing::NodeId{3}, Meshing::NodeId{4});
    twinManager.registerTwin(TwinManager2D::NO_SURFACE, Meshing::NodeId{10}, Meshing::NodeId{20}, TwinManager2D::NO_SURFACE, Meshing::NodeId{30}, Meshing::NodeId{40});

    auto twin1 = twinManager.getTwin(TwinManager2D::NO_SURFACE, Meshing::NodeId{1}, Meshing::NodeId{2});
    ASSERT_TRUE(twin1.has_value());
    EXPECT_EQ(std::get<1>(*twin1), Meshing::NodeId{3});

    auto twin2 = twinManager.getTwin(TwinManager2D::NO_SURFACE, Meshing::NodeId{10}, Meshing::NodeId{20});
    ASSERT_TRUE(twin2.has_value());
    EXPECT_EQ(std::get<1>(*twin2), Meshing::NodeId{30});
    EXPECT_EQ(std::get<2>(*twin2), Meshing::NodeId{40});

    EXPECT_FALSE(twinManager.hasTwin(TwinManager2D::NO_SURFACE, Meshing::NodeId{1}, Meshing::NodeId{10}));
    EXPECT_FALSE(twinManager.hasTwin(TwinManager2D::NO_SURFACE, Meshing::NodeId{3}, Meshing::NodeId{30}));
}

TEST(TwinManager2DTest, CrossSurfaceTwins)
{
    TwinManager2D twinManager;
    twinManager.registerTwin("S1", Meshing::NodeId{1}, Meshing::NodeId{2}, "S2", Meshing::NodeId{3}, Meshing::NodeId{4});

    auto twin = twinManager.getTwin("S1", Meshing::NodeId{1}, Meshing::NodeId{2});
    ASSERT_TRUE(twin.has_value());
    EXPECT_EQ(std::get<0>(*twin), "S2");
    EXPECT_EQ(std::get<1>(*twin), Meshing::NodeId{3});
    EXPECT_EQ(std::get<2>(*twin), Meshing::NodeId{4});

    auto twinReversed = twinManager.getTwin("S2", Meshing::NodeId{3}, Meshing::NodeId{4});
    ASSERT_TRUE(twinReversed.has_value());
    EXPECT_EQ(std::get<0>(*twinReversed), "S1");
    EXPECT_EQ(std::get<1>(*twinReversed), Meshing::NodeId{1});
    EXPECT_EQ(std::get<2>(*twinReversed), Meshing::NodeId{2});

    // Different surface — no twin
    EXPECT_FALSE(twinManager.getTwin("S3", Meshing::NodeId{1}, Meshing::NodeId{2}).has_value());
    EXPECT_FALSE(twinManager.getTwin(TwinManager2D::NO_SURFACE, Meshing::NodeId{1}, Meshing::NodeId{2}).has_value());
}
