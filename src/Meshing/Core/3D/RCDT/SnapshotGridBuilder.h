#pragma once

namespace Export
{
struct VtkGrid;
} // namespace Export

namespace Meshing
{

struct SurfaceMesh3D;

/// Lays RCDT state out as the grids MeshSnapshotWriter writes.
class SnapshotGridBuilder
{
public:
    SnapshotGridBuilder() = delete;

    /// Every node as a point, every triangle as a cell, coloured by the
    /// SurfaceID of the CAD surface it lies on.
    static Export::VtkGrid fromSurfaceMesh(const SurfaceMesh3D& surfaceMesh);
};

} // namespace Meshing
