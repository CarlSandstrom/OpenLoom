#pragma once

#include <cstddef>
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
        numSegmentsPerEdge_(1),
        maxAngleBetweenSegments_(2 * 3.14159265358979323846 / 8)
    {
    }

    explicit DiscretizationSettings2D(std::optional<size_t> numSegmentsPerEdge,
                                      std::optional<double> maxAngleBetweenSegments) :
        numSegmentsPerEdge_(numSegmentsPerEdge),
        maxAngleBetweenSegments_(maxAngleBetweenSegments)
    {
    }

    std::optional<size_t> getNumSegmentsPerEdge() const { return numSegmentsPerEdge_; }
    std::optional<double> getMaxAngleBetweenSegments() const { return maxAngleBetweenSegments_; }

private:
    std::optional<size_t> numSegmentsPerEdge_;
    std::optional<double> maxAngleBetweenSegments_;
};

} // namespace Geometry2D
