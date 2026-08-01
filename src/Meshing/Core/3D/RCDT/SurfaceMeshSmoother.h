#pragma once

#include <cstddef>

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
class SurfaceMeshSmoother
{
public:
    explicit SurfaceMeshSmoother(const Geometry3D::GeometryCollection3D& geometry);

    /// Smooths mesh in place, running the given number of Laplacian sweeps.
    void smooth(SurfaceMesh3D& mesh, std::size_t iterations) const;

private:
    const Geometry3D::GeometryCollection3D* geometry_;
};

} // namespace Meshing
