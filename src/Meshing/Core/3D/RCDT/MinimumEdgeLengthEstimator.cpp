#include "Meshing/Core/3D/RCDT/MinimumEdgeLengthEstimator.h"

#include "Meshing/Core/3D/General/SizingField3D.h"

#include <algorithm>
#include <limits>

namespace Meshing
{

namespace
{

constexpr double MINIMUM_EDGE_LENGTH_DIVISOR = 10.0;

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
            if (i == j)
                continue;
            nearest = std::min(nearest, (points[i] - points[j]).norm());
        }
        nearestPerPoint.push_back(nearest);
    }
    if (nearestPerPoint.empty())
        return 0.0;

    std::sort(nearestPerPoint.begin(), nearestPerPoint.end());
    const double median = nearestPerPoint[nearestPerPoint.size() / 2];
    return median / MINIMUM_EDGE_LENGTH_DIVISOR;
}

double MinimumEdgeLengthEstimator::fromSizingField(const SizingField3D& sizingField)
{
    return sizingField.getMinimumSourceSize() / MINIMUM_EDGE_LENGTH_DIVISOR;
}

} // namespace Meshing
