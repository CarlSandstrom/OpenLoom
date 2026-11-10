#include "Meshing/Core/3D/General/MeshSnapshotWriter.h"

#include "Common/DebugFlags.h"
#include "Export/TsvExporter.h"
#include "Export/VtkExporter.h"
#include "Export/VtkGrid.h"
#include "spdlog/spdlog.h"

#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>

namespace Meshing
{

namespace
{

std::size_t sequence = 0;

std::size_t readStride();
std::string buildStem(const std::string& label);

} // namespace

bool MeshSnapshotWriter::isEnabled()
{
    return OPENLOOM_DEBUG_ENABLED(EXPORT_MESH_EACH_ITERATION);
}

bool MeshSnapshotWriter::isInsertionDue(std::size_t insertionCount)
{
    return isEnabled() && insertionCount % readStride() == 0;
}

void MeshSnapshotWriter::restartSequence()
{
    sequence = 0;
}

void MeshSnapshotWriter::write(const std::string& label, const Export::VtkGrid& grid)
{
    if (!isEnabled()) return;

    auto stem = buildStem(label);
    try
    {
        if (!Export::VtkExporter().writeGrid(grid, stem + ".vtu"))
        {
            spdlog::warn("MeshSnapshotWriter: could not write {}.vtu", stem);
        }
        Export::TsvExporter::writeGrid(grid, stem);
    }
    catch (const std::exception& error)
    {
        spdlog::warn("MeshSnapshotWriter: snapshot {} failed: {}", stem, error.what());
    }
}

namespace
{

std::size_t readStride()
{
    auto text = std::getenv("EXPORT_MESH_EVERY_N");
    if (text == nullptr) return 1;

    char* end = nullptr;
    long long value = std::strtoll(text, &end, 10);
    if (end != text && *end == '\0' && value > 0) return static_cast<std::size_t>(value);

    static std::string warnedValue;
    if (warnedValue != text)
    {
        warnedValue = text;
        spdlog::warn("MeshSnapshotWriter: EXPORT_MESH_EVERY_N='{}' is not a positive integer, using 1", text);
    }
    return 1;
}

std::string buildStem(const std::string& label)
{
    auto directoryText = std::getenv("EXPORT_MESH_DIRECTORY");
    std::filesystem::path directory = directoryText != nullptr ? directoryText : ".";
    std::error_code ignored;
    std::filesystem::create_directories(directory, ignored);

    char number[32];
    std::snprintf(number, sizeof(number), "%06zu", sequence++);
    return (directory / ("rcdt_" + std::string(number) + "_" + label)).string();
}

} // namespace

} // namespace Meshing
