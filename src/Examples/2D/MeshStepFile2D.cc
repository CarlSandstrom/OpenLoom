#include "Export/VtkExporter.h"
#include "Meshing/Core/2D/ConstrainedDelaunay2D.h"
#include "Meshing/Core/2D/EdgeDiscretizer2D.h"
#include "Meshing/Core/2D/MeshingContext2D.h"
#include "Meshing/Core/2D/Mesh2DQualitySettings.h"
#include "Meshing/Core/2D/ShewchukRefiner2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Readers/OpenCascade/StepReader2D.h"
#include "spdlog/spdlog.h"

#include <cmath>
#include <iostream>
#include <string>

using namespace Meshing;

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <step-file>" << std::endl;
        return 1;
    }

    std::string stepFile = argv[1];

    spdlog::set_level(spdlog::level::debug);
    spdlog::set_pattern("[%H:%M:%S] %^%v%$");

    spdlog::info("Loading 2D STEP file: {}", stepFile);

    Readers::StepReader2D reader(stepFile);

    spdlog::info("Topology: {} corners, {} edges",
                 reader.getTopology().getCornerCount(),
                 reader.getTopology().getEdgeCount());

    // Create meshing context (takes ownership of geometry and topology)
    MeshingContext2D context(reader.takeGeometry(), reader.takeTopology());

    // Discretize edges
    EdgeDiscretizer2D discretizer(context);
    auto discretization = discretizer.discretize();

    // Create constrained Delaunay triangulation
    spdlog::info("Generating constrained Delaunay triangulation...");
    ConstrainedDelaunay2D mesher(context, discretization);
    mesher.triangulate();

    // Refine mesh
    spdlog::info("Refining mesh...");
    ShewchukRefiner2D refiner(context, Mesh2DQualitySettings{});
    refiner.refine();

    spdlog::info("Mesh generation complete!");

    Export::VtkExporter exporter;
    exporter.exportMesh(context.getMeshData(), "MeshStepFile2D.vtu");

    return 0;
}
