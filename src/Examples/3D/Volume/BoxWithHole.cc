/**
 * @file BoxWithHole.cc
 * @brief 3D volume mesh example: a box with a cylindrical hole drilled through it.
 *
 * Exports:
 *   - BoxWithHole_mesh.vtu : tetrahedral volume mesh (color by SurfaceID on
 *     boundary triangles; tetrahedra carry SurfaceID = -1)
 */

#include "../Readers/OpenCascade/TopoDS_ShapeConverter.h"
#include "Common/Logging.h"
#include "Export/VtkExporter.h"
#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/Volume/VolumeMesher3D.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <iostream>

int main()
{
    Common::initLogging();

    TopoDS_Shape box = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();

    // Drill a cylindrical hole through the box center using a boolean cut.
    gp_Pnt center(5.0, 5.0, 0.0);
    gp_Dir axisDirection(0.0, 0.0, 1.0);
    gp_Ax2 axis(center, axisDirection);
    TopoDS_Shape cylinder = BRepPrimAPI_MakeCylinder(axis, 2.0, 10.0).Shape();
    TopoDS_Shape boxWithHole = BRepAlgoAPI_Cut(box, cylinder).Shape();

    Readers::TopoDS_ShapeConverter converter(boxWithHole);

    Geometry3D::DiscretizationSettings3D discretizationSettings(3, 2);

    // The sizing field is what keeps this model meshable (OPE-185). Without
    // it the bore's crease and seam are discretized into 2.0-long segments
    // while refinement targets 0.33, so their protecting balls span several
    // elements and freeze a band around the bore that refinement can never
    // enter -- leaving holes in the restricted boundary, which lets the
    // ambient flood fill reach the interior and classify every tetrahedron
    // as ambient. h(x) bounds those segments to the size the mesh actually
    // wants, and the balls shrink with them.
    Meshing::VolumeMesher3D mesher(converter.getGeometryCollection(),
                                   converter.getTopology(),
                                   discretizationSettings,
                                   Meshing::SurfaceMesh3DQualitySettings{},
                                   Meshing::SizingFieldSettings3D{});

    auto volumeMesh = mesher.mesh();

    std::cout << "VolumeMesh3D: " << volumeMesh.nodes.size() << " nodes, "
              << volumeMesh.tetrahedra.size() << " tetrahedra, "
              << volumeMesh.boundaryTriangles.size() << " boundary triangles\n";

    Export::VtkExporter exporter;
    exporter.writeVolumeMesh(volumeMesh, "BoxWithHole_mesh.vtu");
    std::cout << "Exported volume mesh to BoxWithHole_mesh.vtu\n";

    return 0;
}
