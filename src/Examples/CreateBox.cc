#include "../Readers/OpenCascade/TopoDS_ShapeConverter.h"
#include "Export/VtkExporter.h"
#include "Meshing/Core/MeshingContext.h"
#include "Meshing/Core/SimpleMesher.h"
#include <BRepPrimAPI_MakeBox.hxx>
#include <TopoDS_Shape.hxx>

int main()
{
    TopoDS_Shape cube = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();
    Readers::TopoDS_ShapeConverter converter(cube);
    Meshing::MeshingContext context(
        converter.getGeometryCollection(),
        converter.getTopology());

    Meshing::SimpleMesher mesher;
    mesher.generate(context);

    Export::VtkExporter exporter;
    exporter.writeVtu(context.getMeshData(), "box_mesh.vtu");

    return 0;
}