#include "Meshing/Core/3D/General/MeshSnapshotWriter.h"

#include "Common/DebugFlags.h"
#include "Export/TsvExporter.h"
#include "Export/VtkExporter.h"
#include "Meshing/Data/3D/SurfaceMesh3D.h"
#include "spdlog/spdlog.h"

#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <utility>

namespace Meshing
{

namespace
{

size_t readStride()
{
    auto text = std::getenv("EXPORT_MESH_EVERY_N");
    if (text == nullptr) return 1;

    char* end = nullptr;
    long long value = std::strtoll(text, &end, 10);
    if (end == text || *end != '\0' || value <= 0)
    {
        spdlog::warn("MeshSnapshotWriter: EXPORT_MESH_EVERY_N='{}' is not a positive integer, using 1", text);
        return 1;
    }
    return static_cast<size_t>(value);
}

std::string formatSequence(size_t sequence)
{
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%06zu", sequence);
    return buffer;
}

} // namespace

MeshSnapshotWriter::MeshSnapshotWriter(std::string outputDirectory) :
    outputDirectory_(std::move(outputDirectory)),
    enabled_(OPENLOOM_DEBUG_ENABLED(EXPORT_MESH_EACH_ITERATION)),
    stride_(enabled_ ? readStride() : 1)
{
}

bool MeshSnapshotWriter::isEnabled() const
{
    return enabled_;
}

bool MeshSnapshotWriter::isInsertionDue(size_t insertionCount) const
{
    return enabled_ && insertionCount % stride_ == 0;
}

void MeshSnapshotWriter::write(const std::string& label, const SurfaceMesh3D& surfaceMesh)
{
    if (!enabled_) return;

    auto stem = (std::filesystem::path(outputDirectory_) / ("rcdt_" + formatSequence(sequence_++) + "_" + label)).string();

    try
    {
        if (!Export::VtkExporter().writeSurfaceMesh(surfaceMesh, stem + ".vtu"))
        {
            spdlog::warn("MeshSnapshotWriter: could not write {}.vtu", stem);
        }
        Export::TsvExporter::writeSurfaceMesh(surfaceMesh, stem);
    }
    catch (const std::exception& error)
    {
        spdlog::warn("MeshSnapshotWriter: snapshot {} failed: {}", stem, error.what());
    }
}

} // namespace Meshing
