#pragma once

#include "IExporter.h"
#include <iosfwd>
#include <string>
#include <vector>

namespace Meshing
{
class MeshData;
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
    bool exportMesh(const Meshing::MeshData& mesh, const std::string& filePath) const override;
    std::string getName() const override { return "VTK"; }
    std::string getExtension() const override { return "vtu"; }

    // Direct convenience method identical to exportMesh
    bool writeVtu(const Meshing::MeshData& mesh, const std::string& filePath) const { return exportMesh(mesh, filePath); }

private:
    void writeHeader(std::ostream& os) const;
    void writePoints(std::ostream& os, const Meshing::MeshData& mesh) const;
    void writeCells(std::ostream& os, const Meshing::MeshData& mesh) const;
    void writeFooter(std::ostream& os) const;

    static int vtkCellTypeFor(const Meshing::IElement& element);
};

} // namespace Export
