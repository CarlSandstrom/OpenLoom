#include <gtest/gtest.h>

#include "Meshing/Core/3D/General/SizingField3D.h"
#include "Meshing/Core/3D/RCDT/MinimumEdgeLengthEstimator.h"

#include <vector>

using Meshing::MinimumEdgeLengthEstimator;
using Meshing::Point3D;
using Meshing::SizingField3D;
using Meshing::SizingSource;

namespace
{

constexpr double TOLERANCE = 1e-12;

/// A field whose sources all ask for the same size, spread along the x axis.
SizingField3D uniformField(size_t sourceCount, double size)
{
    std::vector<SizingSource> sources;
    sources.reserve(sourceCount);
    for (size_t i = 0; i < sourceCount; ++i)
        sources.push_back({Point3D(static_cast<double>(i), 0.0, 0.0), size});
    return SizingField3D(std::move(sources), 0.3);
}

} // namespace

TEST(MinimumEdgeLengthEstimatorTest, UniformFieldGivesTheCommonSourceSizeOverTheDivisor)
{
    EXPECT_NEAR(MinimumEdgeLengthEstimator::fromSizingField(uniformField(50, 2.0)), 0.2, TOLERANCE);
}

// The defect this estimator's percentile exists for (OPE-180): curvature and
// local-feature-size sampling can emit a few sources asking for elements far
// below anything the rest of the model needs. Reading the single smallest let
// one of them drag the floor -- and with it how far refinement runs -- across
// the whole model.
TEST(MinimumEdgeLengthEstimatorTest, ALoneOutlierSourceDoesNotDragTheFloorDown)
{
    std::vector<SizingSource> sources;
    for (size_t i = 0; i < 50; ++i)
        sources.push_back({Point3D(static_cast<double>(i), 0.0, 0.0), 2.0});
    sources.push_back({Point3D(-1.0, 0.0, 0.0), 0.02});

    const SizingField3D field(std::move(sources), 0.3);

    EXPECT_NEAR(field.getMinimumSourceSize(), 0.02, TOLERANCE);
    EXPECT_NEAR(MinimumEdgeLengthEstimator::fromSizingField(field), 0.2, TOLERANCE);
}

// A genuinely fine region is not an outlier: once enough sources ask for the
// smaller size, the floor follows them down. Without this the percentile would
// be a blunt cap that ignored real refinement demand.
TEST(MinimumEdgeLengthEstimatorTest, AFineRegionLargeEnoughToBeRepresentativeLowersTheFloor)
{
    std::vector<SizingSource> sources;
    for (size_t i = 0; i < 50; ++i)
        sources.push_back({Point3D(static_cast<double>(i), 0.0, 0.0), 2.0});
    for (size_t i = 0; i < 30; ++i)
        sources.push_back({Point3D(-1.0 - static_cast<double>(i), 0.0, 0.0), 0.02});

    const SizingField3D field(std::move(sources), 0.3);

    EXPECT_NEAR(MinimumEdgeLengthEstimator::fromSizingField(field), 0.002, TOLERANCE);
}

TEST(MinimumEdgeLengthEstimatorTest, PointSpacingUsesTheMedianRatherThanTheSmallestGap)
{
    // Four points a unit apart, plus one crowded against the first. The two
    // crowded points each report a 0.01 nearest neighbour, and the point at 1.0
    // reports 0.99 rather than 1.0 because the intruder is now its closest --
    // so the sorted distances are {0.01, 0.01, 0.99, 1.0, 1.0} and the median
    // sits at 0.99, unmoved by the outlier pair.
    const std::vector<Point3D> points = {Point3D(0.0, 0.0, 0.0),
                                         Point3D(1.0, 0.0, 0.0),
                                         Point3D(2.0, 0.0, 0.0),
                                         Point3D(3.0, 0.0, 0.0),
                                         Point3D(0.01, 0.0, 0.0)};

    EXPECT_NEAR(MinimumEdgeLengthEstimator::fromPointSpacing(points), 0.099, TOLERANCE);
}
