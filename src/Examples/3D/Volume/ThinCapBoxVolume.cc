/**
 * @file ThinCapBoxVolume.cc
 * @brief Volume mesh of a box with a 1 mm cap: the volume counterpart of ThinCapBoxSurface.
 *
 * A 10x10x10 box with a 6x6x9 pocket cut from the bottom face leaves a cap 1 mm thick. The
 * tetrahedra spanning the cap join two parallel faces 1 mm apart, a thin feature where slivers
 * form easily.
 *
 * Exports:
 *   - ThinCapBoxVolume_mesh.vtu : tetrahedral volume mesh
 *   - ThinCapBoxVolume_mesh.nodes.tsv, ThinCapBoxVolume_mesh.cells.tsv : the same mesh as tables
 */

#include "Common/Logging.h"
#include "Export/TsvExporter.h"
#include "Export/VtkExporter.h"
#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/Volume/VolumeMesher3D.h"
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

    Meshing::VolumeMesher3D mesher(converter.getGeometryCollection(),
                                   converter.getTopology(),
                                   settings,
                                   Meshing::SurfaceMesh3DQualitySettings{});
    mesher.mesh();
    auto& volumeMesh = *mesher.getMesh();

    std::cout << "VolumeMesh3D: " << volumeMesh.nodes.size() << " nodes, "
              << volumeMesh.tetrahedra.size() << " tetrahedra, "
              << volumeMesh.boundaryTriangles.size() << " boundary triangles\n";

    Export::VtkExporter exporter;
    exporter.writeVolumeMesh(volumeMesh, "ThinCapBoxVolume_mesh.vtu");
    Export::TsvExporter::writeVolumeMesh(volumeMesh, "ThinCapBoxVolume_mesh");
    std::cout << "Exported volume mesh to ThinCapBoxVolume_mesh.vtu\n";

    return 0;
}
