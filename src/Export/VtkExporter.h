#pragma once

#include "IExporter.h"
#include <iosfwd>
#include <string>
#include <vector>

namespace Meshing
{
class MeshData2D;
class MeshData3D;
class IElement;
} // namespace Meshing

namespace Export
{

// Lightweight VTK exporter for unstructured grid (.vtu ASCII)
// Supports points and a subset of cell types (currently Tetra only).
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

    // Overloaded methods for 2D meshes (exported with z=0)
    bool exportMesh(const Meshing::MeshData2D& mesh, const std::string& filePath) const;
    bool writeVtu(const Meshing::MeshData2D& mesh, const std::string& filePath) const { return exportMesh(mesh, filePath); }

private:
    void writeHeader(std::ostream& os) const;
    void writePoints(std::ostream& os, const Meshing::MeshData3D& mesh,
                     std::vector<std::size_t>& outNodeIds) const;
    void writePointData(std::ostream& os, const std::vector<std::size_t>& nodeIds) const;
    void writeCells(std::ostream& os, const Meshing::MeshData3D& mesh,
                    std::vector<std::size_t>& outElementIds) const;
    void writeCellData(std::ostream& os, const std::vector<std::size_t>& elementIds) const;
    void writeFooter(std::ostream& os) const;

    // Convert 2D mesh to 3D (with z=0) for export
    static Meshing::MeshData3D convertToMeshData3D(const Meshing::MeshData2D& mesh2D);

    static int vtkCellTypeFor(const Meshing::IElement& element);
};

} // namespace Export
