#include "Meshing/Core/3D/General/MeshDebugUtils3D.h"
#include "Common/DebugFlags.h"
#include "Export/VtkExporter.h"
#include "Meshing/Core/3D/General/DiscretizationResult3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/SurfaceMesh3D.h"
#include "spdlog/spdlog.h"

namespace Meshing
{

void exportMesh3D(MeshData3D& meshData, const std::string& filenamePrefix, size_t exportCounter)
{
    if (OPENLOOM_DEBUG_ENABLED(EXPORT_MESH_EACH_ITERATION))
    {
        Export::VtkExporter exporter;
        exporter.exportMesh(meshData, filenamePrefix + "_" + std::to_string(exportCounter) + ".vtu");
    }
}

void exportEdgeMesh3D(const DiscretizationResult3D& result, const std::string& filename)
{
    if (OPENLOOM_DEBUG_ENABLED(EXPORT_MESH_EACH_ITERATION))
    {
        Export::VtkExporter exporter;
        exporter.writeEdgeMesh(result, filename);
        spdlog::info("MeshDebugUtils3D: exported edge mesh to {}", filename);
    }
}

void exportSurfaceMesh3D(const SurfaceMesh3D& surfaceMesh, const std::string& filename)
{
    if (OPENLOOM_DEBUG_ENABLED(EXPORT_MESH_EACH_ITERATION))
    {
        Export::VtkExporter exporter;
        exporter.writeSurfaceMesh(surfaceMesh, filename);
        spdlog::info("MeshDebugUtils3D: exported surface mesh to {}", filename);
    }
}

} // namespace Meshing
