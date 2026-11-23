/**
 * @file CreateBox.cc
 * @brief Basic 3D volume mesh example: a plain box, no holes.
 *
 * Exports:
 *   - box_mesh.vtu : tetrahedral volume mesh (color by SurfaceID on boundary
 *     triangles; tetrahedra carry SurfaceID = -1)
 *   - box_mesh.nodes.tsv, box_mesh.cells.tsv : the same mesh as tab-separated tables
 */

#include "Common/Logging.h"
#include "Export/TsvExporter.h"
#include "Export/VtkExporter.h"
#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/Volume/VolumeMesher3D.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"
#include "Readers/OpenCascade/TopoDS_ShapeConverter.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <TopoDS_Shape.hxx>
#include <iostream>

namespace
{
constexpr double BOX_SIDE_LENGTH = 10.0;
} // namespace

int main()
{
    Common::initializeLogging();

    TopoDS_Shape box = BRepPrimAPI_MakeBox(BOX_SIDE_LENGTH, BOX_SIDE_LENGTH, BOX_SIDE_LENGTH).Shape();
    Readers::TopoDS_ShapeConverter converter(box);

    Geometry3D::DiscretizationSettings3D discretizationSettings(3, 2);

    Meshing::VolumeMesher3D mesher(converter.getGeometryCollection(),
                                   converter.getTopology(),
                                   discretizationSettings,
                                   Meshing::SurfaceMesh3DQualitySettings{});

    mesher.mesh();
    auto& volumeMesh = *mesher.getMesh();

    std::cout << "VolumeMesh3D: " << volumeMesh.nodes.size() << " nodes, "
              << volumeMesh.tetrahedra.size() << " tetrahedra, "
              << volumeMesh.boundaryTriangles.size() << " boundary triangles\n";

    Export::VtkExporter exporter;
    exporter.writeVolumeMesh(volumeMesh, "box_mesh.vtu");
    Export::TsvExporter::writeVolumeMesh(volumeMesh, "box_mesh");
    std::cout << "Exported volume mesh to box_mesh.vtu and box_mesh.{nodes,cells}.tsv\n";

    return 0;
}
