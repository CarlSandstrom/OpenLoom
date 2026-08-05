#pragma once

#include <cstddef>
#include <optional>

namespace Meshing
{

/**
 * @brief Quality settings shared by both 3D surface meshing strategies:
 * the UV-space pipeline (SurfaceMeshingContext3D) and ambient-space RCDT
 * (RCDTMesher). All angle parameters are in degrees.
 */
struct SurfaceMesh3DQualitySettings
{
    /// Maximum allowed circumradius / shortest-edge ratio.
    /// A bound of 2.0 corresponds roughly to a 30° minimum angle. RCDT's
    /// termination guarantee has been tuned and tested against 1.0 (see
    /// RCDTRefiner), which is why that — not 2.0 — is the default here.
    double circumradiusToShortestEdgeRatio = 1.0;

    /// Minimum interior angle of any triangle, in degrees.
    double minAngleDegrees = 30.0;

    /// Safety cap: stop refining a face once it reaches this many elements.
    std::size_t elementLimit = 50000;

    /// Maximum allowed chord height between a flat triangle and the CAD surface,
    /// in model units. Set to 0 to disable chord-deviation checking.
    double chordDeviationTolerance = 0.1;

    /// Laplacian smoothing sweeps applied to the output surface mesh after
    /// refinement (see SurfaceMeshSmoother). Only consumed by RCDTMesher —
    /// Delaunay refinement alone does not reliably converge to FEM-quality
    /// elements on curved surfaces. Set to 0 to disable.
    std::size_t smoothingIterations = 5;

    /// Floor on a restricted triangle's shortest edge below which it is left
    /// unrefined even if it still fails the quality criteria above. Only
    /// consumed by RCDTMesher. Without this, a triangle whose ratio sits just
    /// past the threshold can have its circumcenter land within roughly one
    /// edge-length of its own vertices, producing an equally-bad, slightly
    /// smaller sliver next to it every iteration — a non-terminating cascade.
    /// If unset, RCDTRefiner derives it from the initial boundary
    /// discretization: the median nearest-neighbor distance among the
    /// initial nodes, divided by 10.
    std::optional<double> minimumEdgeLength;

    /// Maximum allowed tetrahedron circumradius / shortest-edge ratio
    /// (Shewchuk's "B" bound; only guarantees refinement termination for
    /// B > 2.0). Only consumed by RCDTMesher::meshVolume() — the surface-only
    /// path (meshSurface()) never looks at this.
    double tetCircumradiusToShortestEdgeRatio = 2.5;

    /// Safety cap: stop volume refinement once the mesh reaches this many
    /// tetrahedra. Separate from elementLimit (which bounds restricted
    /// surface triangles) since the two element counts aren't comparable —
    /// a solid's interior typically needs far more tets than it has boundary
    /// triangles.
    std::size_t tetElementLimit = 100000;

    /// Maximum number of refinement iterations before the RCDT refiner gives
    /// up. The default (500) is a safety cap for ordinary meshes. Geometries
    /// with acute dihedral angles at feature corners (< 60°) drive a
    /// segment-splitting cascade that terminates correctly via the minimum
    /// edge length floor but requires more iterations. Increase this for
    /// stress tests or geometries with known acute input angles. Only consumed
    /// by RCDTMesher — the UV-space pipeline ignores it.
    std::size_t maxRefinementIterations = 50000;
};

} // namespace Meshing
