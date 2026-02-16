#pragma once

#include <cstddef>
#include <string>

namespace Meshing
{

class MeshData2D;

/**
 * @brief Conditionally export and verify a 2D mesh based on debug flags.
 *
 * When EXPORT_MESH_EACH_ITERATION is enabled, exports the mesh to a VTU file
 * named "{filenamePrefix}_{counter}.vtu" and increments the counter.
 * When CHECK_MESH_EACH_ITERATION is enabled, runs MeshVerifier and throws
 * on failure.
 *
 * @param meshData The mesh data to export/verify
 * @param filenamePrefix Prefix for the exported VTU filename
 * @param exportCounter Counter tracking export iterations (incremented on export)
 */
void exportAndVerifyMesh(MeshData2D& meshData,
                         const std::string& filenamePrefix,
                         size_t& exportCounter);

} // namespace Meshing
