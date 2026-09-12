#pragma once

#include <array>
#include <cstddef>
#include <vector>

namespace Meshing
{
struct SurfaceMesh3D;
} // namespace Meshing

namespace Geometry3D
{
class GeometryCollection3D;
} // namespace Geometry3D

namespace Meshing
{

/// Post-processing quality pass for RCDT output surface meshes.
///
/// Moves each interior (non-boundary) vertex toward the centroid of its mesh
/// neighbors and re-projects it onto its owning CAD surface, repeated for a
/// fixed number of iterations. Vertices on a CAD edge or corner (present in
/// any SurfaceMesh3D::edgeNodeIds entry) are never moved, since their
/// position is fixed by the boundary curve they belong to.
///
/// Delaunay refinement on a curved surface does not reliably converge to
/// FEM-quality elements on its own (see RCDTRefiner) — this smoothing pass
/// is the standard follow-up production meshers (Gmsh, Netgen) use to
/// improve minimum angle without changing mesh topology.
///
/// Each iteration is two phases. The first proposes a new position for every
/// movable node, all read off the positions the iteration started from. The
/// second hands the proposal to a TetrahedronInversionGuard: when the surface
/// bounds a volume mesh its nodes are shared with the tetrahedra, and a move
/// that is harmless for the surface can turn an adjacent tetrahedron inside
/// out, so the guard undoes the moves that would. With no tetrahedra to
/// guard, the proposal is taken as it stands.
class SurfaceMeshSmoother
{
public:
    /// Smooths mesh in place, running the given number of Laplacian sweeps.
    /// tetrahedra index into mesh.nodes and must not be inverted by any sweep;
    /// empty when the surface bounds no volume mesh.
    static void smooth(const Geometry3D::GeometryCollection3D& geometry,
                       SurfaceMesh3D& mesh,
                       std::size_t iterations,
                       const std::vector<std::array<std::size_t, 4>>& tetrahedra);
};

} // namespace Meshing
