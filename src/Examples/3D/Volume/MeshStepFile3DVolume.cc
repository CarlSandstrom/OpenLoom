/**
 * @file MeshStepFile3DVolume.cc
 * @brief Volume-mesh any 3D STEP file (OPE-177 stress test).
 *
 * Generic tool, not tied to one part: takes a STEP file path and runs it
 * through VolumeMesher3D with default settings. VolumeMesher3D::mesh()
 * refuses a restricted boundary with holes, so a successful run is itself
 * proof the surface came out watertight.
 */

#include "Common/Logging.h"
#include "Export/VtkExporter.h"
#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/Volume/VolumeMesher3D.h"
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

    Meshing::VolumeMesher3D mesher(converter.getGeometryCollection(),
                                   converter.getTopology(),
                                   discretizationSettings,
                                   Meshing::SurfaceMesh3DQualitySettings{});
    auto volumeMesh = mesher.mesh();

    std::cout << "VolumeMesh3D: " << volumeMesh.nodes.size() << " nodes, "
              << volumeMesh.tetrahedra.size() << " tetrahedra, "
              << volumeMesh.boundaryTriangles.size() << " boundary triangles\n";

    const std::string outputName = std::filesystem::path(stepFile).stem().string() + "_volume.vtu";
    Export::VtkExporter exporter;
    exporter.writeVolumeMesh(volumeMesh, outputName);
    std::cout << "Exported volume mesh to " << outputName << "\n";

    return 0;
}
