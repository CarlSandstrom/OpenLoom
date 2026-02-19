#include "../Readers/OpenCascade/TopoDS_ShapeConverter.h"
#include "Export/VtkExporter.h"
#include "Meshing/Core/3D/General/MeshingContext3D.h"
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

int main()
{
    TopoDS_Shape box = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();

    // Drill a cylindrical hole through the box center using a boolean cut.
    gp_Pnt center(5.0, 5.0, 0.0);
    gp_Dir axisDirection(0.0, 0.0, 1.0);
    gp_Ax2 axis(center, axisDirection);
    TopoDS_Shape cylinder = BRepPrimAPI_MakeCylinder(axis, 2.0, 10.0).Shape();
    TopoDS_Shape boxWithHole = BRepAlgoAPI_Cut(box, cylinder).Shape();

    Readers::TopoDS_ShapeConverter converter(boxWithHole);
    Meshing::MeshingContext3D context(
        converter.getGeometryCollection(),
        converter.getTopology());

    // TODO: Implement 3D meshing algorithm
    // Meshing::ConstrainedDelaunay3D mesher(context);
    // mesher.generateConstrained(1, 5);

    Export::VtkExporter exporter;
    exporter.writeVtu(context.getMeshData(), "BoxWithHole_mesh.vtu");

    return 0;
}