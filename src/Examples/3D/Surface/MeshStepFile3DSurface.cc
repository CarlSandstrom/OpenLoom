/**
 * @file MeshStepFile3DSurface.cc
 * @brief Surface-mesh any 3D STEP file (OPE-177 stress test).
 *
 * Generic tool, not tied to one part: takes a STEP file path and runs it
 * through SurfaceMesher3D with default settings, reporting node/triangle
 * counts on success.
 */

#include "Common/Logging.h"
#include "Export/VtkExporter.h"
#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/Surface/SurfaceMesher3D.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"
#include "Readers/OpenCascade/StepReader3D.h"
#include "Readers/OpenCascade/TopoDS_ShapeConverter.h"

#include <filesystem>
#include <iostream>
#include <numbers>
#include <spdlog/spdlog.h>
#include <string>

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <step-file>" << std::endl;
        return 1;
    }

    Common::initializeLogging();

    const std::string stepFile = argv[1];
    spdlog::info("Loading 3D STEP file: {}", stepFile);

    Readers::StepReader3D reader(stepFile);
    Readers::TopoDS_ShapeConverter converter(reader.getShape());

    Geometry3D::DiscretizationSettings3D discretizationSettings(std::nullopt, std::numbers::pi / 8.0, 2);

    Meshing::SurfaceMesher3D mesher(converter.getGeometryCollection(),
                                    converter.getTopology(),
                                    discretizationSettings,
                                    Meshing::SurfaceMesh3DQualitySettings{});
    auto surfaceMesh = mesher.mesh();

    std::cout << "SurfaceMesh3D: " << surfaceMesh.nodes.size() << " nodes, "
              << surfaceMesh.triangles.size() << " triangles\n";

    const std::string outputName = std::filesystem::path(stepFile).stem().string() + "_surface.vtu";
    Export::VtkExporter exporter;
    exporter.writeSurfaceMesh(surfaceMesh, outputName);
    std::cout << "Exported surface mesh to " << outputName << "\n";

    return 0;
}
