#pragma once

#include <cstddef>
#include <optional>

namespace Meshing
{

/**
 * @brief Quality settings for the 3D surface and volume meshers (RCDTMesher).
 * All angle parameters are in degrees.
 */
struct SurfaceMesh3DQualitySettings
{
    /// Minimum interior angle of any triangle, in degrees.
    double minAngleDegrees = 30.0;

    /// Maximum allowed chord height between a flat triangle and the CAD surface,
    /// in model units. Set to 0 to disable chord-deviation checking.
    double chordDeviationTolerance = 0.1;

    /// Laplacian smoothing sweeps applied to the output surface mesh after
    /// refinement (see SurfaceMeshSmoother). Delaunay refinement alone does
    /// not reliably converge to FEM-quality elements on curved surfaces. Set
    /// to 0 to disable.
    std::size_t smoothingIterations = 5;

    /// Floor on a restricted triangle's shortest edge below which it is left
    /// unrefined even if it still fails the quality criteria above. Without
    /// this, a triangle whose ratio sits just past the threshold can have its
    /// circumcenter land within roughly one
    /// edge-length of its own vertices, producing an equally-bad, slightly
    /// smaller sliver next to it every iteration — a non-terminating cascade.
    /// If unset, RCDTMesher derives it (see MinimumEdgeLengthEstimator); if
    /// set, it must be finite and strictly positive.
    std::optional<double> minimumEdgeLength;

    /// Maximum allowed tetrahedron circumradius / shortest-edge ratio
    /// (Shewchuk's "B" bound; only guarantees refinement termination for
    /// B > 2.0). Only consumed by RCDTMesher::meshVolume() — the surface-only
    /// path (meshSurface()) never looks at this.
    double tetCircumradiusToShortestEdgeRatio = 2.5;

    /// Maximum number of refinement iterations before the RCDT refiner gives
    /// up. The default (500) is a safety cap for ordinary meshes. Geometries
    /// with acute dihedral angles at feature corners (< 60°) drive a
    /// segment-splitting cascade that terminates correctly via the minimum
    /// edge length floor but requires more iterations. Increase this for
    /// stress tests or geometries with known acute input angles.
    std::size_t maxRefinementIterations = 50000;
};

} // namespace Meshing
