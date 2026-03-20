/**
 * @file SurfaceMeshEdges.cc
 * @brief S1 validation: initialize surface meshing context and export boundary edge and surface mesh.
 *
 * Demonstrates the S1 pipeline (TwinTableGenerator → BoundaryDiscretizer3D →
 * FacetTriangulationManager) on a box-with-hole CAD shape and exports:
 *   - SurfaceMeshEdges.vtu        : discretized boundary edges (color by EdgeID)
 *   - SurfaceMesh3D.vtu           : initial surface triangulation (color by SurfaceID)
 *   - SurfaceMesh3D_Refined.vtu   : refined surface triangulation (color by SurfaceID)
 *
 * Open both files in ParaView to verify each topology edge and surface triangulation.
 */

#include "../Readers/OpenCascade/TopoDS_ShapeConverter.h"
#include "Export/VtkExporter.h"
#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/Surface/SurfaceMesh3DQualitySettings.h"
#include "Meshing/Core/3D/General/DiscretizationResult3D.h"
#include "Meshing/Core/3D/General/FacetTriangulationManager.h"
#include "Meshing/Core/3D/Surface/SurfaceMesher3D.h"
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
    // Build box-with-cylindrical-hole CAD shape (same geometry as BoxWithHole)
    TopoDS_Shape box = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();

    gp_Pnt center(7.50, 7.50, 0.0);
    gp_Dir axisDirection(0.0, 0.0, 1.0);
    gp_Ax2 axis(center, axisDirection);
    TopoDS_Shape cylinder = BRepPrimAPI_MakeCylinder(axis, 2.0, 20.0).Shape();
    TopoDS_Shape shape = BRepAlgoAPI_Cut(box, cylinder).Shape();

    // Convert CAD shape to geometry + topology
    Readers::TopoDS_ShapeConverter converter(shape);

    // Discretization settings: angle-based — inserts points only where the tangent
    // direction changes by more than π/8 (22.5°). Straight edges (box edges, seam)
    // produce no interior points; circular arc edges are resolved adaptively.
    Geometry3D::DiscretizationSettings3D settings(std::nullopt, std::numbers::pi / 8.0, 2);

    // S1 pipeline: TwinTableGenerator → BoundaryDiscretizer3D → FacetTriangulationManager
    Meshing::SurfaceMeshingContext3D context(converter.getGeometryCollection(),
                                             converter.getTopology(),
                                             settings,
                                             Meshing::SurfaceMesh3DQualitySettings{});

    const auto& discResult = context.getDiscretizationResult();
    const auto subfacets = context.getFacetTriangulationManager().getAllSubfacets();

    std::cout << "Points:         " << discResult.points.size() << "\n";
    std::cout << "Topology edges: " << discResult.edgeIdToPointIndicesMap.size() << "\n";
    std::cout << "Faces:          " << context.getFacetTriangulationManager().size() << "\n";
    std::cout << "Subfacets:      " << subfacets.size() << "\n";

    Export::VtkExporter exporter;

    // Export discretized edges — color by EdgeID to verify shared-edge alignment
    exporter.writeEdgeMesh(discResult, "SurfaceMeshEdges.vtu");
    std::cout << "Exported edge mesh to SurfaceMeshEdges.vtu (color by EdgeID)\n";

    // Export initial surface triangulation — color by SurfaceID
    exporter.writeSurfaceMesh(discResult, subfacets, "SurfaceMesh3D.vtu");
    std::cout << "Exported surface mesh to SurfaceMesh3D.vtu (color by SurfaceID)\n";

    // S2–S3: Refine each face in UV space using Shewchuk's algorithm
    context.refineSurfaces();
    auto surfaceMesh = context.buildSurfaceMesh();

    std::cout << "Refined: " << surfaceMesh.nodes.size() << " nodes, "
              << surfaceMesh.triangles.size() << " triangles\n";

    // Export refined surface triangulation
    exporter.writeSurfaceMesh(surfaceMesh, "SurfaceMesh3D_Refined.vtu");
    std::cout << "Exported refined surface mesh to SurfaceMesh3D_Refined.vtu\n";

    return 0;
}
