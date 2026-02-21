#include "MeshDebugUtils2D.h"
#include "Common/DebugFlags.h"
#include "Common/Exceptions/MeshException.h"
#include "Export/VtkExporter.h"
#include "Meshing/Core/2D/MeshVerifier.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "spdlog/spdlog.h"

namespace Meshing
{

void exportAndVerifyMesh(MeshData2D& meshData,
                         const std::string& filenamePrefix,
                         size_t& exportCounter)
{
    if (OPENLOOM_DEBUG_ENABLED(EXPORT_MESH_EACH_ITERATION))
    {
        Export::VtkExporter exporter;
        exporter.exportMesh(meshData, filenamePrefix + "_" + std::to_string(exportCounter++) + ".vtu");
    }

    if (OPENLOOM_DEBUG_ENABLED(CHECK_MESH_EACH_ITERATION))
    {
        spdlog::info("{}: Verifying mesh at export step {}", filenamePrefix, exportCounter);
        MeshVerifier verifier(meshData);

        auto result = verifier.verify();
        if (!result.isValid)
        {
            for (const auto& error : result.errors)
            {
                spdlog::error(" - {}", error);
            }
            OPENLOOM_THROW_VERIFICATION_FAILED("Mesh verification failed", result.errors);
        }
    }
}

} // namespace Meshing
