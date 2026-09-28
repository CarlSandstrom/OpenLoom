#pragma once

#include <cstddef>

namespace Meshing
{

/**
 * @brief Quality settings for 2D Delaunay mesh refinement.
 *
 * Passed to ShewchukRefiner2D to control when triangles are acceptable.
 * All angle parameters are in degrees.
 */
struct Mesh2DQualitySettings
{
    /// Maximum allowed circumradius / shortest-edge ratio.
    /// A bound of 2.0 corresponds roughly to a 30° minimum angle.
    double circumradiusToEdgeRatio = 2.0;

    /// Minimum interior angle of any triangle, in degrees.
    double minAngleDegrees = 30.0;

    /// Safety cap: stop refining once the mesh reaches this many elements.
    std::size_t elementLimit = 10000;
};

} // namespace Meshing
