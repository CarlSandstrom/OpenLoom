#pragma once

#include <cstddef>
#include <numbers>
#include <optional>

namespace Geometry3D
{

/**
 * @brief Settings for discretizing 3D geometry into mesh points
 *
 * Encapsulates parameters that control how geometric entities (edges, surfaces)
 * are converted into discrete point sets for meshing.
 *
 * Edge discretization supports two modes (angle-based takes priority):
 *  - Angle-based: inserts a point whenever the tangent direction changes by
 *    more than maxAngleBetweenSegments. Straight edges produce no interior
 *    points; curved edges are resolved adaptively.
 *  - Fixed-count: divides every edge into numberOfSegmentsPerEdge uniform segments.
 */
class DiscretizationSettings3D
{
public:
    /**
     * @brief Default constructor — angle-based with π/4 (45°), 2 surface samples.
     */
    DiscretizationSettings3D() :
        numberOfSegmentsPerEdge_(std::nullopt),
        maxAngleBetweenSegments_(std::numbers::pi / 4.0),
        numberOfSamplesPerSurfaceDirection_(2)
    {
    }

    /**
     * @brief Full explicit constructor.
     * @param numberOfSegmentsPerEdge   Fixed segment count (nullopt = not used).
     * @param maxAngleBetweenSegments  Max tangent-angle change per segment (nullopt = not used).
     * @param numberOfSamplesPerSurfaceDirection  Grid samples per direction on surfaces.
     */
    DiscretizationSettings3D(std::optional<size_t> numberOfSegmentsPerEdge,
                             std::optional<double> maxAngleBetweenSegments,
                             size_t numberOfSamplesPerSurfaceDirection) :
        numberOfSegmentsPerEdge_(numberOfSegmentsPerEdge),
        maxAngleBetweenSegments_(maxAngleBetweenSegments),
        numberOfSamplesPerSurfaceDirection_(numberOfSamplesPerSurfaceDirection)
    {
    }

    /**
     * @brief Convenience constructor for fixed-count mode (backwards compatible).
     * @param numberOfSegmentsPerEdge Number of segments to divide each edge into.
     * @param numberOfSamplesPerSurfaceDirection Grid samples per direction on surfaces.
     */
    DiscretizationSettings3D(size_t numberOfSegmentsPerEdge,
                             size_t numberOfSamplesPerSurfaceDirection) :
        numberOfSegmentsPerEdge_(numberOfSegmentsPerEdge),
        maxAngleBetweenSegments_(std::nullopt),
        numberOfSamplesPerSurfaceDirection_(numberOfSamplesPerSurfaceDirection)
    {
    }

    std::optional<size_t> getNumberOfSegmentsPerEdge() const { return numberOfSegmentsPerEdge_; }
    std::optional<double> getMaxAngleBetweenSegments() const { return maxAngleBetweenSegments_; }
    size_t getNumberOfSamplesPerSurfaceDirection() const { return numberOfSamplesPerSurfaceDirection_; }

private:
    std::optional<size_t> numberOfSegmentsPerEdge_;
    std::optional<double> maxAngleBetweenSegments_;
    size_t numberOfSamplesPerSurfaceDirection_;
};

} // namespace Geometry3D
