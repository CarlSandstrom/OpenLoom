#include <gtest/gtest.h>

#include "Common/Types.h"
#include "Meshing/Core/2D/ConstraintChecker2D.h"
#include "Meshing/Core/2D/GeometryStructures2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/MeshMutator2D.h"

using namespace Meshing;

namespace
{
constexpr double TOLERANCE = 1e-9;
}

class ConstraintChecker2DTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        mutator_ = std::make_unique<MeshMutator2D>(meshData_);
    }

    size_t addNode(double x, double y)
    {
        return mutator_->addNode(Point2D(x, y));
    }

    MeshData2D meshData_;
    std::unique_ptr<MeshMutator2D> mutator_;
};

TEST_F(ConstraintChecker2DTest, IsSegmentEncroachedDetectsPointInDiametralCircle)
{
    size_t n0 = addNode(0.0, 0.0);
    size_t n1 = addNode(4.0, 0.0);

    ConstraintChecker2D checker(meshData_);
    ConstrainedSegment2D segment{n0, n1};

    Point2D inside(2.0, 0.5);
    Point2D outside(2.0, 3.0);

    EXPECT_TRUE(checker.isSegmentEncroached(segment, inside));
    EXPECT_FALSE(checker.isSegmentEncroached(segment, outside));
}

TEST_F(ConstraintChecker2DTest, IsSegmentEncroachedReturnsFalseForEndpoints)
{
    size_t n0 = addNode(0.0, 0.0);
    size_t n1 = addNode(4.0, 0.0);

    ConstraintChecker2D checker(meshData_);
    ConstrainedSegment2D segment{n0, n1};

    Point2D endpoint1(0.0, 0.0);
    Point2D endpoint2(4.0, 0.0);

    // Endpoints are on the boundary, not inside
    EXPECT_FALSE(checker.isSegmentEncroached(segment, endpoint1));
    EXPECT_FALSE(checker.isSegmentEncroached(segment, endpoint2));
}

TEST_F(ConstraintChecker2DTest, IsSegmentEncroachedForDiagonalSegment)
{
    size_t n0 = addNode(0.0, 0.0);
    size_t n1 = addNode(2.0, 2.0);

    ConstraintChecker2D checker(meshData_);
    ConstrainedSegment2D segment{n0, n1};

    Point2D inside(1.0, 1.0);  // Center of segment, definitely inside
    Point2D outside(3.0, 0.0); // Far from segment

    EXPECT_TRUE(checker.isSegmentEncroached(segment, inside));
    EXPECT_FALSE(checker.isSegmentEncroached(segment, outside));
}
