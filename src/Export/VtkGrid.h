#pragma once

#include "Common/Types.h"

#include <cstddef>
#include <string>
#include <variant>
#include <vector>

namespace Export
{

enum class VtkCellType
{
    Line = 3,
    Triangle = 5,
    Quadrilateral = 9,
    Tetrahedron = 10,
    Hexahedron = 12,
    Wedge = 13,
    Pyramid = 14
};

struct VtkCell
{
    VtkCellType type;
    std::vector<std::size_t> pointIndices;
};

/// One value per point, or one per cell, under a name ParaView can colour by.
struct VtkField
{
    std::string name;
    std::variant<std::vector<int>, std::vector<std::size_t>, std::vector<double>> values;
};

/**
 * @brief An unstructured grid assembled by the caller, for
 * VtkExporter::writeGrid to serialize.
 *
 * For views that are not a mesh the exporter already knows how to walk --
 * a subset of faces, a field computed by a diagnostic -- so the caller can
 * build exactly what it wants to look at without writing VTU itself.
 * nodeIds is written as the `NodeID` point array, parallel to points; cells
 * index into points.
 */
struct VtkGrid
{
    std::vector<Meshing::Point3D> points;
    std::vector<std::size_t> nodeIds;
    std::vector<VtkCell> cells;
    std::vector<VtkField> pointFields;
    std::vector<VtkField> cellFields;
};

} // namespace Export
