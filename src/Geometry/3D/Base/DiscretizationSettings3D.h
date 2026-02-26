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
 *  - Fixed-count: divides every edge into numSegmentsPerEdge uniform segments.
 */
class DiscretizationSettings3D
{
public:
    /**
     * @brief Default constructor — angle-based with π/4 (45°), 2 surface samples.
     */
    DiscretizationSettings3D() :
        numSegmentsPerEdge_(std::nullopt),
        maxAngleBetweenSegments_(std::numbers::pi / 4.0),
        numSamplesPerSurfaceDirection_(2)
    {
    }

    /**
     * @brief Full explicit constructor.
     * @param numSegmentsPerEdge   Fixed segment count (nullopt = not used).
     * @param maxAngleBetweenSegments  Max tangent-angle change per segment (nullopt = not used).
     * @param numSamplesPerSurfaceDirection  Grid samples per direction on surfaces.
     */
    DiscretizationSettings3D(std::optional<size_t> numSegmentsPerEdge,
                              std::optional<double> maxAngleBetweenSegments,
                              size_t numSamplesPerSurfaceDirection) :
        numSegmentsPerEdge_(numSegmentsPerEdge),
        maxAngleBetweenSegments_(maxAngleBetweenSegments),
        numSamplesPerSurfaceDirection_(numSamplesPerSurfaceDirection)
    {
    }

    /**
     * @brief Convenience constructor for fixed-count mode (backwards compatible).
     * @param numSegmentsPerEdge Number of segments to divide each edge into.
     * @param numSamplesPerSurfaceDirection Grid samples per direction on surfaces.
     */
    DiscretizationSettings3D(size_t numSegmentsPerEdge,
                              size_t numSamplesPerSurfaceDirection) :
        numSegmentsPerEdge_(numSegmentsPerEdge),
        maxAngleBetweenSegments_(std::nullopt),
        numSamplesPerSurfaceDirection_(numSamplesPerSurfaceDirection)
    {
    }

    std::optional<size_t> getNumSegmentsPerEdge() const { return numSegmentsPerEdge_; }
    std::optional<double> getMaxAngleBetweenSegments() const { return maxAngleBetweenSegments_; }
    size_t getNumSamplesPerSurfaceDirection() const { return numSamplesPerSurfaceDirection_; }

private:
    std::optional<size_t> numSegmentsPerEdge_;
    std::optional<double> maxAngleBetweenSegments_;
    size_t numSamplesPerSurfaceDirection_;
};

} // namespace Geometry3D
