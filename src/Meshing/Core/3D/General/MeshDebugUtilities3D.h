#pragma once

#include <cstddef>
#include <string>

namespace Meshing
{

class MeshData3D;
class DiscretizationResult3D;
struct SurfaceMesh3D;

/**
 * @brief Conditionally export a 3D mesh to VTU when EXPORT_MESH_EACH_ITERATION is enabled.
 *
 * Writes "{filenamePrefix}_{counter}.vtu" and increments the counter.
 *
 * @param meshData The mesh data to export
 * @param filenamePrefix Prefix for the exported VTU filename
 * @param exportCounter Counter used as the filename index (not modified).
 */
void exportMesh3D(MeshData3D& meshData,
                  const std::string& filenamePrefix,
                  size_t exportCounter);

/**
 * @brief Conditionally export discretized boundary edges based on EXPORT_MESH_EACH_ITERATION.
 *
 * Exports the edge mesh to "{filename}" when the flag is enabled.
 *
 * @param result    The discretization result containing edge points and indices
 * @param filename  Output VTU filename
 */
void exportEdgeMesh3D(const DiscretizationResult3D& result, const std::string& filename);

/**
 * @brief Conditionally export a triangle-only surface mesh based on EXPORT_MESH_EACH_ITERATION.
 *
 * Unlike exportMesh3D, which dumps the full ambient tetrahedralization (including
 * interior tets whose faces were never selected as part of the boundary), this
 * writes only the triangles actually chosen as output — the same faces
 * RCDTMesher::mesh() returns. Triangles are colored by their CAD surface ID.
 *
 * @param surfaceMesh The assembled surface mesh to export
 * @param filename    Output VTU filename
 */
void exportSurfaceMesh3D(const SurfaceMesh3D& surfaceMesh, const std::string& filename);

} // namespace Meshing
