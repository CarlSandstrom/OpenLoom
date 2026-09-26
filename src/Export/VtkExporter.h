#pragma once

#include "IExporter.h"
#include <string>
#include <vector>

namespace Meshing
{
class MeshData2D;
class MeshData3D;
struct DiscretizationResult3D;
struct ConstrainedSubfacet3D;
struct SurfaceMesh3D;
struct VolumeMesh3D;
} // namespace Meshing

namespace Export
{
struct VtkGrid;

// ASCII .vtu writer for viewing meshes in ParaView. Every method lays its
// input out as a VtkGrid and hands it to writeGrid. Output is not a
// behavioural contract -- the goldens are TsvExporter's -- so fields may be
// added or changed freely.
class VtkExporter : public IExporter
{
public:
    VtkExporter() = default;

    // IExporter implementation
    bool exportMesh(const Meshing::MeshData3D& mesh, const std::string& filePath) const override;
    std::string getName() const override { return "VTK"; }
    std::string getExtension() const override { return "vtu"; }

    // Direct convenience method identical to exportMesh
    bool writeVtu(const Meshing::MeshData3D& mesh, const std::string& filePath) const { return exportMesh(mesh, filePath); }

    // Export discretized boundary edges as VTK_LINE cells.
    // Each line cell carries an EdgeID scalar (0-based index over topology edges)
    // for color-by-edge inspection in ParaView.
    bool writeEdgeMesh(const Meshing::DiscretizationResult3D& result, const std::string& filePath) const;

    // Export surface triangulation as VTK_TRIANGLE cells.
    // Each triangle cell carries a SurfaceID scalar (0-based index over unique surface IDs)
    // for color-by-surface inspection in ParaView.
    bool writeSurfaceMesh(const Meshing::DiscretizationResult3D& discretization,
                          const std::vector<Meshing::ConstrainedSubfacet3D>& subfacets,
                          const std::string& filePath) const;

    // Overload that takes a fully assembled SurfaceMesh3D (output of SurfaceMesher3D).
    // Triangles are coloured by SurfaceID using the per-face groups in the struct.
    bool writeSurfaceMesh(const Meshing::SurfaceMesh3D& surfaceMesh,
                          const std::string& filePath) const;

    // Export a fully assembled VolumeMesh3D (output of VolumeMesher3D).
    // Tetrahedra are written as VTK_TETRA cells; boundary triangles are
    // written as VTK_TRIANGLE cells coloured by SurfaceID.
    bool writeVolumeMesh(const Meshing::VolumeMesh3D& volumeMesh,
                         const std::string& filePath) const;

    // Export a caller-assembled grid: arbitrary cells plus named point and
    // cell fields, for views that are not one of the meshes above.
    bool writeGrid(const VtkGrid& grid, const std::string& filePath) const;

    // Overloaded methods for 2D meshes (exported with z=0)
    bool exportMesh(const Meshing::MeshData2D& mesh, const std::string& filePath) const;
    bool writeVtu(const Meshing::MeshData2D& mesh, const std::string& filePath) const { return exportMesh(mesh, filePath); }

};

} // namespace Export
