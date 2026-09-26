#pragma once

#include <string>
#include <vector>

namespace Meshing
{
class MeshData2D;
struct ConstrainedSubfacet3D;
struct DiscretizationResult3D;
struct SurfaceMesh3D;
struct VolumeMesh3D;
} // namespace Meshing

namespace Export
{
struct VtkGrid;

/**
 * @brief Writes mesher output as tab-separated tables: the format
 * `scripts/refactor-check.sh` diffs against `tests/golden/`.
 *
 * Every call writes two files next to each other:
 *
 *  - `<stem>.nodes.tsv` -- one row per node: `node_id x y z`, plus any
 *    per-node attributes the output carries.
 *  - `<stem>.cells.tsv` -- one row per cell: `kind id nodes geometry_id role`,
 *    where `nodes` is a comma-separated node id list and `-` marks a column
 *    that does not apply to that kind.
 *
 * This is the behavioural contract, not a view. It carries exactly what the
 * mesher produced, in a canonical order, with coordinates written as
 * shortest round-trip doubles -- so any change in output is a diff, and a
 * diff names the rows that moved. Fields that only aid inspection, or that
 * the exporter derives from the mesh (VtkExporter's DomainID, its 0-based
 * SurfaceID/EdgeID recolouring), do not belong here: VtkExporter is the view
 * and is free to change without moving a golden.
 */
class TsvExporter
{
public:
    static void writeMesh(const Meshing::MeshData2D& mesh, const std::string& stem);

    static void writeDiscretization(const Meshing::DiscretizationResult3D& discretization,
                                    const std::string& stem);

    static void writeSurfaceMesh(const Meshing::DiscretizationResult3D& discretization,
                                 const std::vector<Meshing::ConstrainedSubfacet3D>& subfacets,
                                 const std::string& stem);

    static void writeSurfaceMesh(const Meshing::SurfaceMesh3D& surfaceMesh, const std::string& stem);

    static void writeVolumeMesh(const Meshing::VolumeMesh3D& volumeMesh, const std::string& stem);

    /// The same caller-assembled grid VtkExporter::writeGrid draws, as
    /// tables: each point and cell field becomes a column. Cells list node
    /// ids, not point indices. For diagnostics, not goldens.
    static void writeGrid(const VtkGrid& grid, const std::string& stem);
};

} // namespace Export
