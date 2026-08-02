/**
 * @file CreateBox.cc
 * @brief Basic 3D volume mesh example: a plain box, no holes.
 *
 * Exports:
 *   - box_mesh.vtu : tetrahedral volume mesh (color by SurfaceID on boundary
 *     triangles; tetrahedra carry SurfaceID = -1)
 */

#include "../Readers/OpenCascade/TopoDS_ShapeConverter.h"
#include "Common/Logging.h"
#include "Export/VtkExporter.h"
#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/Volume/VolumeMesher3D.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <TopoDS_Shape.hxx>
#include <iostream>

int main()
{
    Common::initLogging();

    TopoDS_Shape cube = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();
    Readers::TopoDS_ShapeConverter converter(cube);

    Geometry3D::DiscretizationSettings3D discretizationSettings(3, 2);

    Meshing::VolumeMesher3D mesher(converter.getGeometryCollection(),
                                   converter.getTopology(),
                                   discretizationSettings,
                                   Meshing::SurfaceMesh3DQualitySettings{});

    auto volumeMesh = mesher.mesh();

    std::cout << "VolumeMesh3D: " << volumeMesh.nodes.size() << " nodes, "
              << volumeMesh.tetrahedra.size() << " tetrahedra, "
              << volumeMesh.boundaryTriangles.size() << " boundary triangles\n";

    Export::VtkExporter exporter;
    exporter.writeVolumeMesh(volumeMesh, "box_mesh.vtu");
    std::cout << "Exported volume mesh to box_mesh.vtu\n";

    return 0;
}
