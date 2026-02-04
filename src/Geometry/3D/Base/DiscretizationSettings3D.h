#pragma once

#include <cstddef>
#include <optional>

namespace Geometry3D
{

/**
 * @brief Settings for discretizing 3D geometry into mesh points
 *
 * Encapsulates parameters that control how geometric entities (edges, surfaces)
 * are converted into discrete point sets for meshing.
 */
class DiscretizationSettings3D
{
public:
    /**
     * @brief Default constructor with sensible defaults
     *
     * Initializes with 3 segments per edge and 2x2 samples per surface.
     */
    DiscretizationSettings3D() :
        numSegmentsPerEdge_(3),
        numSamplesPerSurfaceDirection_(2)
    {
    }

    /**
     * @brief Construct with explicit settings
     * @param numSegmentsPerEdge Number of segments to divide each edge into
     * @param numSamplesPerSurfaceDirection Grid samples per direction on surfaces
     */
    DiscretizationSettings3D(size_t numSegmentsPerEdge,
                              size_t numSamplesPerSurfaceDirection) :
        numSegmentsPerEdge_(numSegmentsPerEdge),
        numSamplesPerSurfaceDirection_(numSamplesPerSurfaceDirection)
    {
    }

    size_t getNumSegmentsPerEdge() const { return numSegmentsPerEdge_; }
    size_t getNumSamplesPerSurfaceDirection() const { return numSamplesPerSurfaceDirection_; }

    void setNumSegmentsPerEdge(size_t value) { numSegmentsPerEdge_ = value; }
    void setNumSamplesPerSurfaceDirection(size_t value) { numSamplesPerSurfaceDirection_ = value; }

private:
    size_t numSegmentsPerEdge_;
    size_t numSamplesPerSurfaceDirection_;
};

} // namespace Geometry3D
