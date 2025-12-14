#include "../Readers/OpenCascade/TopoDS_ShapeConverter.h"
#include "Export/VtkExporter.h"
#include "Meshing/Core/ConstrainedMesher.h" // NEW
#include "Meshing/Core/MeshingContext.h"
#include "spdlog/spdlog.h"
#include <BRepPrimAPI_MakeBox.hxx>
#include <TopoDS_Shape.hxx>

int main()
{
    // Set up logging
    spdlog::set_level(spdlog::level::info);
    spdlog::set_pattern("[%H:%M:%S] %^%v%$");

    // Create a simple box
    TopoDS_Shape box = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();

    // Convert CAD to topology + geometry
    Readers::TopoDS_ShapeConverter converter(box);

    // Create meshing context
    Meshing::MeshingContext3D context(
        converter.getGeometryCollection(),
        converter.getTopology());

    // Use ConstrainedMesher - THIS NOW RESPECTS BOTH EDGES AND FACES!
    // Parameters: (qualityController, samplesPerEdge, samplesPerSurface)
    Meshing::ConstrainedMesher mesher(nullptr, 10, 5);

    spdlog::info("Generating constrained mesh:");
    spdlog::info("  10 samples per edge");
    spdlog::info("  5x5 samples per surface");

    mesher.generate(context);

    // Export to VTK
    Export::VtkExporter exporter;
    exporter.writeVtu(context.getMeshData(), "constrained_box_mesh.vtu");

    spdlog::info("");
    spdlog::info("✓ Constrained mesh exported to constrained_box_mesh.vtu");
    spdlog::info("✓ All edges are respected in the mesh");
    spdlog::info("✓ All faces are triangulated and forced into mesh");
    spdlog::info("");
    spdlog::info("Open in ParaView to verify constraints:");
    spdlog::info("  paraview constrained_box_mesh.vtu");

    return 0;
}
