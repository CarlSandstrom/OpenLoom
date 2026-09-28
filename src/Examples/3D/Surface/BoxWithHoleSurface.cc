/**
 * @file BoxWithHoleSurface.cc
 * @brief Surface mesh of a 10×10×10 box with a cylindrical hole drilled through its center.
 *
 * Geometry matches the BoxWithHole volume example: a unit box cut by a cylinder
 * of radius 2 centred at (5, 5) along Z.
 *
 * Exports:
 *   - BoxWithHoleSurfaceEdges.vtu : discretized boundary edges (color by EdgeID)
 *   - BoxWithHoleSurface3D.vtu    : surface mesh (color by SurfaceID)
 */

#include "../Readers/OpenCascade/TopoDS_ShapeConverter.h"
#include "Common/Logging.h"
#include "Export/TsvExporter.h"
#include "Export/VtkExporter.h"
#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/General/BoundaryDiscretizer3D.h"
#include "Meshing/Core/3D/Surface/SurfaceMesher3D.h"
#include "Meshing/Data/3D/DiscretizationResult3D.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <iostream>
#include <numbers>

int main()
{
    Common::initLogging();

    TopoDS_Shape box = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();

    gp_Pnt center(5.0, 5.0, 0.0);
    gp_Dir axisDirection(0.0, 0.0, 1.0);
    gp_Ax2 axis(center, axisDirection);
    TopoDS_Shape cylinder = BRepPrimAPI_MakeCylinder(axis, 2.0, 10.0).Shape();
    TopoDS_Shape shape = BRepAlgoAPI_Cut(box, cylinder).Shape();

    Readers::TopoDS_ShapeConverter converter(shape);

    Geometry3D::DiscretizationSettings3D settings(std::nullopt, std::numbers::pi / 8.0, 2);

    const auto discResult = Meshing::BoundaryDiscretizer3D::discretize(converter.getGeometryCollection(),
                                                                       converter.getTopology(),
                                                                       settings);

    std::cout << "Points:         " << discResult->points.size() << "\n";
    std::cout << "Topology edges: " << discResult->edgeIdToPointIndicesMap.size() << "\n";
    std::cout << "Faces:          " << converter.getTopology().getAllSurfaceIds().size() << "\n";

    Export::VtkExporter exporter;

    exporter.writeEdgeMesh(*discResult, "BoxWithHoleSurfaceEdges.vtu");
    Export::TsvExporter::writeDiscretization(*discResult, "BoxWithHoleSurfaceEdges");
    std::cout << "Exported edge mesh to BoxWithHoleSurfaceEdges.vtu (color by EdgeID)\n";

    Meshing::SurfaceMesher3D mesher(converter.getGeometryCollection(),
                                    converter.getTopology(),
                                    settings,
                                    Meshing::SurfaceMesh3DQualitySettings{});
    auto surfaceMesh = mesher.mesh();
    std::cout << "SurfaceMesh3D: " << surfaceMesh.nodes.size() << " nodes, "
              << surfaceMesh.triangles.size() << " triangles\n";

    exporter.writeSurfaceMesh(surfaceMesh, "BoxWithHoleSurface3D.vtu");
    Export::TsvExporter::writeSurfaceMesh(surfaceMesh, "BoxWithHoleSurface3D");
    std::cout << "Exported surface mesh to BoxWithHoleSurface3D.vtu (color by SurfaceID)\n";

    return 0;
}
