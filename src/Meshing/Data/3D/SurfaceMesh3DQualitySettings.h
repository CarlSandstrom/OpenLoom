#pragma once

#include <cstddef>

namespace Meshing
{

/**
 * @brief Quality settings for 3D surface mesh refinement.
 *
 * Passed to SurfaceMeshingContext3D to control both the angle-quality pass
 * (Phase 1) and the optional chord-deviation pass (Phase 2).
 * All angle parameters are in degrees.
 */
struct SurfaceMesh3DQualitySettings
{
    /// Maximum allowed circumradius / shortest-edge ratio (Phase 1).
    /// A bound of 2.0 corresponds roughly to a 30° minimum angle.
    double circumradiusToEdgeRatio = 2.0;

    /// Minimum interior angle of any triangle, in degrees (Phase 1).
    double minAngleDegrees = 30.0;

    /// Safety cap: stop refining a face once it reaches this many elements.
    std::size_t elementLimit = 50000;

    /// Maximum allowed chord height between a flat triangle and the CAD surface,
    /// in model units (Phase 2). Set to 0 to disable the chord-deviation pass.
    double chordDeviationTolerance = 0.0;
};

} // namespace Meshing
