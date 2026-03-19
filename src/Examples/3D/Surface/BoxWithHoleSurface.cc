/**
 * @file BoxWithHoleSurface.cc
 * @brief Surface mesh of a 10×10×10 box with a cylindrical hole drilled through its center.
 *
 * Geometry matches the BoxWithHole volume example: a unit box cut by a cylinder
 * of radius 2 centred at (5, 5) along Z.
 *
 * Exports:
 *   - BoxWithHoleSurfaceEdges.vtu        : discretized boundary edges (color by EdgeID)
 *   - BoxWithHoleSurface3D.vtu           : initial surface triangulation (color by SurfaceID)
 *   - BoxWithHoleSurface3D_Refined.vtu   : refined surface triangulation
 */

#include "../Readers/OpenCascade/TopoDS_ShapeConverter.h"
#include "Export/VtkExporter.h"
#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/Surface/SurfaceMesh3DQualitySettings.h"
#include "Meshing/Core/3D/General/DiscretizationResult3D.h"
#include "Meshing/Core/3D/General/FacetTriangulationManager.h"
#include "Meshing/Core/3D/Surface/SurfaceMeshingContext3D.h"
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
    TopoDS_Shape box = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();

    gp_Pnt center(5.0, 5.0, 0.0);
    gp_Dir axisDirection(0.0, 0.0, 1.0);
    gp_Ax2 axis(center, axisDirection);
    TopoDS_Shape cylinder = BRepPrimAPI_MakeCylinder(axis, 2.0, 10.0).Shape();
    TopoDS_Shape shape = BRepAlgoAPI_Cut(box, cylinder).Shape();

    Readers::TopoDS_ShapeConverter converter(shape);

    Geometry3D::DiscretizationSettings3D settings(std::nullopt, std::numbers::pi / 8.0, 2);

    Meshing::SurfaceMeshingContext3D context(converter.getGeometryCollection(),
                                             converter.getTopology(),
                                             settings,
                                             Meshing::SurfaceMesh3DQualitySettings{});

    const auto& discResult = context.getDiscretizationResult();

    std::cout << "Points:         " << discResult.points.size() << "\n";
    std::cout << "Topology edges: " << discResult.edgeIdToPointIndicesMap.size() << "\n";
    std::cout << "Faces:          " << context.getFacetTriangulationManager().size() << "\n";

    Export::VtkExporter exporter;

    exporter.writeEdgeMesh(discResult, "BoxWithHoleSurfaceEdges.vtu");
    std::cout << "Exported edge mesh to BoxWithHoleSurfaceEdges.vtu (color by EdgeID)\n";

    const auto subfacets = context.getFacetTriangulationManager().getAllSubfacets();
    exporter.writeSurfaceMesh(discResult, subfacets, "BoxWithHoleSurface3D.vtu");
    std::cout << "Exported surface mesh to BoxWithHoleSurface3D.vtu (color by SurfaceID)\n";

    context.refineSurfaces();

    const auto subfacetsRefined = context.getFacetTriangulationManager().getAllSubfacets();
    auto meshData = context.getSurfaceMesh3D();
    std::cout << "Subfacets (refined): " << subfacetsRefined.size() << "\n";
    std::cout << "MeshData3D: " << meshData.getNodeCount() << " nodes, "
              << meshData.getElementCount() << " triangles\n";

    exporter.writeSurfaceMesh(meshData, subfacetsRefined, "BoxWithHoleSurface3D_Refined.vtu");
    std::cout << "Exported refined surface mesh to BoxWithHoleSurface3D_Refined.vtu\n";

    return 0;
}
