#include "../Readers/OpenCascade/TopoDS_ShapeConverter.h"
#include "Export/VtkExporter.h"
#include "Meshing/Core/3D/MeshingContext3D.h"
#include "Meshing/Core/3D/SimpleMesher.h"
#include "spdlog/spdlog.h"
#include <BRepPrimAPI_MakeBox.hxx>
#include <TopoDS_Shape.hxx>

int main()
{
    spdlog::set_level(spdlog::level::trace);
    spdlog::set_pattern("[%H:%M:%S] %s: %^%v%$");

    TopoDS_Shape cube = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();
    Readers::TopoDS_ShapeConverter converter(cube);
    Meshing::MeshingContext3D context(
        converter.getGeometryCollection(),
        converter.getTopology());

    Meshing::SimpleMesher mesher;
    mesher.generate(context);

    Export::VtkExporter exporter;
    exporter.writeVtu(context.getMeshData(), "box_mesh.vtu");

    return 0;
}