#pragma once

#include <cstddef>

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
     * @brief Construct with default settings (1 segment per edge, no subdivision)
     */
    DiscretizationSettings2D() = default;

    /**
     * @brief Construct with specified number of segments per edge
     * @param numSegmentsPerEdge Number of segments to divide each edge into (minimum 1)
     */
    explicit DiscretizationSettings2D(size_t numSegmentsPerEdge)
        : numSegmentsPerEdge_(numSegmentsPerEdge)
    {
    }

    /**
     * @brief Get the number of segments per edge
     */
    size_t getNumSegmentsPerEdge() const { return numSegmentsPerEdge_; }

    // Future constructor overloads:
    // DiscretizationSettings2D(size_t numSegments, double maxAngle);
    // DiscretizationSettings2D(size_t numSegments, double maxAngle, double maxLength);

private:
    size_t numSegmentsPerEdge_ = 1;  ///< Number of segments per edge (default: 1, no subdivision)

    // Future parameters:
    // double maxAngleBetweenSegments_ = M_PI / 4;  // 45 degrees
    // double maxSegmentLength_ = std::numeric_limits<double>::max();
    // double minSegmentLength_ = 0.0;
    // bool adaptiveRefinement_ = false;
};

} // namespace Geometry2D
