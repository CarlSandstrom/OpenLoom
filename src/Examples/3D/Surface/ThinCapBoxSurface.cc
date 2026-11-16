/**
 * @file ThinCapBoxSurface.cc
 * @brief Smallest reproduction of a curve closer to another surface than its protecting balls are wide.
 *
 * A 10x10x10 box with a 6x6x9 pocket cut from the bottom face leaves a cap 1 mm thick. The pocket's
 * ceiling edges lie 1 mm below the box's top face, so protecting balls sized from the edge
 * discretization reach through the cap and the top face meshes with facets that are not flat.
 *
 * Exports:
 *   - ThinCapBoxSurface3D.vtu : surface mesh (color by SurfaceID)
 *   - ThinCapBoxSurface.nodes.tsv, ThinCapBoxSurface.cells.tsv : the same mesh as tables
 */

#include "Common/Logging.h"
#include "Export/TsvExporter.h"
#include "Export/VtkExporter.h"
#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/Surface/SurfaceMesher3D.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"
#include "Readers/OpenCascade/TopoDS_ShapeConverter.h"
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <iostream>
#include <numbers>

int main()
{
    Common::initializeLogging();

    TopoDS_Shape box = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();
    TopoDS_Shape pocket = BRepPrimAPI_MakeBox(gp_Pnt(2.0, 2.0, 0.0), 6.0, 6.0, 9.0).Shape();
    TopoDS_Shape shape = BRepAlgoAPI_Cut(box, pocket).Shape();

    Readers::TopoDS_ShapeConverter converter(shape);

    Geometry3D::DiscretizationSettings3D settings(std::nullopt, std::numbers::pi / 8.0, 2);

    Meshing::SurfaceMesher3D mesher(converter.getGeometryCollection(),
                                    converter.getTopology(),
                                    settings,
                                    Meshing::SurfaceMesh3DQualitySettings{});
    mesher.mesh();
    auto& surfaceMesh = *mesher.getMesh();
    std::cout << "SurfaceMesh3D: " << surfaceMesh.nodes.size() << " nodes, "
              << surfaceMesh.triangles.size() << " triangles\n";

    Export::VtkExporter exporter;
    exporter.writeSurfaceMesh(surfaceMesh, "ThinCapBoxSurface3D.vtu");
    std::cout << "Exported surface mesh to ThinCapBoxSurface3D.vtu (color by SurfaceID)\n";
    Export::TsvExporter::writeSurfaceMesh(surfaceMesh, "ThinCapBoxSurface");

    return 0;
}
