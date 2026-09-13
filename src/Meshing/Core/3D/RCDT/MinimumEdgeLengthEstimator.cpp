#include "Meshing/Core/3D/RCDT/MinimumEdgeLengthEstimator.h"

#include "Meshing/Core/3D/General/SizingField3D.h"

#include <algorithm>
#include <limits>

namespace Meshing
{

namespace
{

constexpr double MINIMUM_EDGE_LENGTH_DIVISOR = 10.0;

// Which of the sizing field's sources the floor is derived from, as a
// fraction through the sorted source sizes. The SMALLEST overbids: curvature
// and local-feature-size sampling routinely produces a handful of sources
// asking for elements far below anything the rest of the model needs, and a
// floor taken from that one source lets refinement chase the same depth
// everywhere else. A LOW percentile keeps the floor below every size the
// field realistically asks for while ignoring those few outliers.
//
// It must stay low: the floor is a sliver guard, not a size target (see the
// header), so a central percentile turns it into one and stops refinement
// where the mesh has arrived. Measured on SaddleSurfaceMesh with the field
// enabled, against 1 non-manifold edge for that model with no field at all
// (OPE-180): 0.05 -> floor 0.0504, 1 defect; 0.10 -> 0.0559, 1; 0.25 ->
// 0.0841, 7; 0.50 -> 0.173, 12; the old global minimum -> 0.0258, 9. The
// plateau spans roughly 0.05-0.10, and 0.1 sits inside it rather than on
// either shoulder.
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
    const auto& sources = sizingField.getSources();

    std::vector<double> sourceSizes;
    sourceSizes.reserve(sources.size());
    for (const SizingSource& source : sources)
        sourceSizes.push_back(source.size);

    // SizingField3D's constructor rejects an empty source list, so this is a
    // guard against a future caller rather than a reachable state today.
    if (sourceSizes.empty())
        return 0.0;

    std::sort(sourceSizes.begin(), sourceSizes.end());
    const size_t percentileIndex =
        std::min(sourceSizes.size() - 1, static_cast<size_t>(SOURCE_SIZE_PERCENTILE * sourceSizes.size()));
    return sourceSizes[percentileIndex] / MINIMUM_EDGE_LENGTH_DIVISOR;
}

} // namespace Meshing
