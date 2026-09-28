#pragma once

#include <cstddef>
#include <numbers>
#include <optional>

namespace Geometry2D
{

/**
 * @brief Settings for discretizing 2D geometry into mesh points
 *
 * Encapsulates parameters that control how geometric entities (edges, curves)
 * are converted into discrete point sets for meshing.
 */
class DiscretizationSettings2D
{
public:
    /**
     * @brief Default constructor with sensible defaults
     *
     * Initializes with 1 segment per edge and max angle of π/4 radians (45°)
     */
    DiscretizationSettings2D() :
        numberOfSegmentsPerEdge_(1),
        maxAngleBetweenSegments_(std::numbers::pi / 4.0)
    {
    }

    explicit DiscretizationSettings2D(std::optional<size_t> numberOfSegmentsPerEdge,
                                      std::optional<double> maxAngleBetweenSegments) :
        numberOfSegmentsPerEdge_(numberOfSegmentsPerEdge),
        maxAngleBetweenSegments_(maxAngleBetweenSegments)
    {
    }

    std::optional<size_t> getNumberOfSegmentsPerEdge() const { return numberOfSegmentsPerEdge_; }
    std::optional<double> getMaxAngleBetweenSegments() const { return maxAngleBetweenSegments_; }

private:
    std::optional<size_t> numberOfSegmentsPerEdge_;
    std::optional<double> maxAngleBetweenSegments_;
};

} // namespace Geometry2D
