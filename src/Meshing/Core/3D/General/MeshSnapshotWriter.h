#pragma once

#include <cstddef>
#include <string>

namespace Export
{
struct VtkGrid;
} // namespace Export

namespace Meshing
{

/**
 * @brief Writes a numbered series of debug snapshots while a mesher runs.
 *
 * Active only when EXPORT_MESH_EACH_ITERATION=1. Two more environment
 * variables are read on every call; this is the only place that reads any of
 * the three:
 *
 *  - EXPORT_MESH_EVERY_N: insertion stride (positive integer, default 1; an
 *    invalid value falls back to 1).
 *  - EXPORT_MESH_DIRECTORY: where snapshots go (default: the working
 *    directory).
 *
 * Each write() produces `rcdt_<sequence>_<label>` in every format the writer
 * knows (VTU for ParaView, TSV for diffing). The sequence is a six-digit
 * counter shared by the whole process, so the series sorts; a mesher run calls
 * restartSequence() first so each run numbers from 0. A failed write is
 * logged and never aborts meshing.
 *
 * The writer knows nothing about what a grid shows: callers assemble it.
 */
class MeshSnapshotWriter
{
public:
    MeshSnapshotWriter() = delete;

    static bool isEnabled();

    /// True when enabled and insertionCount is a multiple of the stride.
    static bool isInsertionDue(std::size_t insertionCount);

    /// The next write() is numbered 0.
    static void restartSequence();

    /// No-op when disabled.
    static void write(const std::string& label, const Export::VtkGrid& grid);
};

} // namespace Meshing
