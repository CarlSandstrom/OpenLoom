#pragma once

#include <string>

namespace Meshing
{
class MeshData;
}

namespace Export
{

// Generic exporter interface for writing mesh data to an external format.
class IExporter
{
public:
    virtual ~IExporter() = default;

    // Export the provided mesh to the given file path. Returns true on success.
    virtual bool exportMesh(const Meshing::MeshData& mesh, const std::string& filePath) const = 0;

    // Human readable name (e.g., "VTK")
    virtual std::string getName() const = 0;

    // Recommended file extension (without leading dot), e.g. "vtu".
    virtual std::string getExtension() const = 0;
};

} // namespace Export
