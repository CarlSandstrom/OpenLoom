#pragma once

#include <cstddef>
#include <string>

namespace Meshing
{

struct SurfaceMesh3D;

/**
 * @brief Writes a numbered series of surface-mesh snapshots while a mesher runs.
 *
 * Active only when EXPORT_MESH_EACH_ITERATION=1. EXPORT_MESH_EVERY_N sets the
 * insertion stride (positive integer, default 1; an invalid value falls back
 * to 1). Both are read once, at construction -- this is the only place that
 * reads them.
 *
 * Each write() produces `<directory>/rcdt_<sequence>_<label>` in every
 * format the writer knows (VTU for ParaView, TSV for diffing). The sequence is
 * a six-digit counter owned by the writer, so the series sorts. A failed write
 * is logged and never aborts meshing.
 */
class MeshSnapshotWriter
{
public:
    explicit MeshSnapshotWriter(std::string outputDirectory = ".");

    bool isEnabled() const;

    /// True when enabled and insertionCount is a multiple of the stride.
    bool isInsertionDue(size_t insertionCount) const;

    /// No-op when disabled.
    void write(const std::string& label, const SurfaceMesh3D& surfaceMesh);

private:
    std::string outputDirectory_;
    bool enabled_;
    size_t stride_;
    size_t sequence_ = 0;
};

} // namespace Meshing
