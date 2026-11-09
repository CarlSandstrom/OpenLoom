#include "Meshing/Core/3D/RCDT/MinimumEdgeLengthEstimator.h"

#include "Meshing/Core/3D/General/SizingField3D.h"

#include <algorithm>
#include <limits>

namespace Meshing
{

namespace
{

constexpr double MINIMUM_EDGE_LENGTH_DIVISOR = 10.0;

// Fraction through the sorted source sizes that the floor is taken from. The
// smallest source overbids, since curvature and feature-size sampling produce a
// few outliers asking for far smaller elements than the model needs. A central
// percentile turns the floor into a size target and stops refinement early.
//
// This is a fitted constant: it was found by sweeping defect counts on
// SaddleSurfaceMesh (OPE-180), the method OPE-208 rules out. Re-derive it from a
// property of the field, such as how many sources are sampling artifacts.
constexpr double SOURCE_SIZE_PERCENTILE = 0.1;

} // namespace

double MinimumEdgeLengthEstimator::fromPointSpacing(const std::vector<Point3D>& points)
{
    std::vector<double> nearestPerPoint;
    nearestPerPoint.reserve(points.size());
    for (size_t i = 0; i < points.size(); ++i)
    {
        double nearest = std::numeric_limits<double>::max();
        for (size_t j = 0; j < points.size(); ++j)
        {
            if (i == j) continue;
            nearest = std::min(nearest, (points[i] - points[j]).norm());
        }
        nearestPerPoint.push_back(nearest);
    }
    if (nearestPerPoint.empty()) return 0.0;

    std::sort(nearestPerPoint.begin(), nearestPerPoint.end());
    double median = nearestPerPoint[nearestPerPoint.size() / 2];
    return median / MINIMUM_EDGE_LENGTH_DIVISOR;
}

double MinimumEdgeLengthEstimator::fromSizingField(const SizingField3D& sizingField)
{
    auto& sources = sizingField.getSources();

    std::vector<double> sourceSizes;
    sourceSizes.reserve(sources.size());
    for (auto& source : sources)
        sourceSizes.push_back(source.size);

    // Unreachable today: SizingField3D's constructor rejects an empty source list.
    if (sourceSizes.empty()) return 0.0;

    std::sort(sourceSizes.begin(), sourceSizes.end());
    size_t percentileIndex =
        std::min(sourceSizes.size() - 1, static_cast<size_t>(SOURCE_SIZE_PERCENTILE * sourceSizes.size()));
    return sourceSizes[percentileIndex] / MINIMUM_EDGE_LENGTH_DIVISOR;
}

} // namespace Meshing
