#include <gtest/gtest.h>

#include "Common/Exceptions/MeshException.h"
#include "Meshing/Core/3D/General/SizingField3D.h"

#include <cmath>
#include <random>
#include <vector>

using Meshing::Point3D;
using Meshing::SizingField3D;
using Meshing::SizingSource;

namespace
{

constexpr double TOLERANCE = 1e-12;

} // namespace

TEST(SizingField3DTest, SingleSourceGrowsLinearlyWithDistance)
{
    SizingField3D field({{Point3D(0.0, 0.0, 0.0), 0.5}}, 0.3);

    EXPECT_NEAR(field.evaluate(Point3D(0.0, 0.0, 0.0)), 0.5, TOLERANCE);
    EXPECT_NEAR(field.evaluate(Point3D(2.0, 0.0, 0.0)), 0.5 + 0.3 * 2.0, TOLERANCE);
    EXPECT_NEAR(field.evaluate(Point3D(0.0, -4.0, 0.0)), 0.5 + 0.3 * 4.0, TOLERANCE);
}

TEST(SizingField3DTest, FinestSourceWinsWhereItIsCloser)
{
    // A coarse source at the origin and a much finer one 10 away: near the
    // fine source the field must follow the fine one, and far from it the
    // coarse one, with the crossover strictly between them.
    SizingField3D field({{Point3D(0.0, 0.0, 0.0), 2.0}, {Point3D(10.0, 0.0, 0.0), 0.1}}, 0.25);

    EXPECT_NEAR(field.evaluate(Point3D(10.0, 0.0, 0.0)), 0.1, TOLERANCE);
    EXPECT_NEAR(field.evaluate(Point3D(0.0, 0.0, 0.0)), 2.0, TOLERANCE);
    EXPECT_LT(field.evaluate(Point3D(5.0, 0.0, 0.0)), 2.0);
}

TEST(SizingField3DTest, NoSourceIsEverExceededAtItsOwnPosition)
{
    // The envelope must not relax a source's own constraint, however many
    // other coarser sources surround it.
    const std::vector<SizingSource> sources = {{Point3D(0.0, 0.0, 0.0), 1.0},
                                               {Point3D(1.0, 0.0, 0.0), 3.0},
                                               {Point3D(0.0, 1.0, 0.0), 0.2},
                                               {Point3D(0.0, 0.0, 1.0), 5.0}};
    SizingField3D field(sources, 0.4);

    for (const auto& source : sources)
    {
        EXPECT_LE(field.evaluate(source.position), source.size + TOLERANCE);
    }
}

TEST(SizingField3DTest, GradientLimitHoldsBetweenArbitraryPointPairs)
{
    // The defining property: |h(a) - h(b)| <= g * |a - b| everywhere, which
    // is what makes the field usable as a graded size target without any
    // separate limiting pass.
    constexpr double GRADIENT_LIMIT = 0.35;
    SizingField3D field({{Point3D(0.0, 0.0, 0.0), 1.0},
                         {Point3D(4.0, 1.0, -2.0), 0.05},
                         {Point3D(-3.0, 2.0, 1.0), 0.6},
                         {Point3D(1.0, -5.0, 3.0), 2.0}},
                        GRADIENT_LIMIT);

    std::mt19937 generator(20260830);
    std::uniform_real_distribution<double> coordinate(-8.0, 8.0);

    for (int sample = 0; sample < 2000; ++sample)
    {
        const Point3D a(coordinate(generator), coordinate(generator), coordinate(generator));
        const Point3D b(coordinate(generator), coordinate(generator), coordinate(generator));

        const double difference = std::abs(field.evaluate(a) - field.evaluate(b));
        EXPECT_LE(difference, GRADIENT_LIMIT * (a - b).norm() + TOLERANCE);
    }
}

TEST(SizingField3DTest, MinimumSourceSizeReportsTheFinestConstraint)
{
    SizingField3D field({{Point3D(0.0, 0.0, 0.0), 1.0},
                         {Point3D(1.0, 0.0, 0.0), 0.125},
                         {Point3D(2.0, 0.0, 0.0), 0.75}},
                        0.3);

    EXPECT_NEAR(field.getMinimumSourceSize(), 0.125, TOLERANCE);
}

TEST(SizingField3DTest, RejectsDegenerateConstruction)
{
    EXPECT_THROW(SizingField3D({}, 0.3), OpenLoom::MeshException);
    EXPECT_THROW(SizingField3D({{Point3D(0.0, 0.0, 0.0), 1.0}}, 0.0), OpenLoom::MeshException);
    EXPECT_THROW(SizingField3D({{Point3D(0.0, 0.0, 0.0), 0.0}}, 0.3), OpenLoom::MeshException);
}
