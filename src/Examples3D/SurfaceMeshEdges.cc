/**
 * @file SurfaceMeshEdges.cc
 * @brief S1 validation: initialize surface meshing context and export boundary edge discretization.
 *
 * Demonstrates the S1 pipeline (TwinTableGenerator → BoundaryDiscretizer3D →
 * FacetTriangulationManager) on a box-with-hole CAD shape and exports the
 * discretized boundary edges as a VTK edge mesh for inspection in ParaView.
 *
 * Color by EdgeID in ParaView to verify each topology edge is discretized
 * correctly and shared-edge segments align between adjacent faces.
 */

#include "../Readers/OpenCascade/TopoDS_ShapeConverter.h"
#include "Export/VtkExporter.h"
#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/General/DiscretizationResult3D.h"
#include "Meshing/Core/3D/Surface/FacetTriangulationManager.h"
#include "Meshing/Core/3D/Surface/SurfaceMeshingContext3D.h"
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
    // Build box-with-cylindrical-hole CAD shape (same geometry as BoxWithHole)
    TopoDS_Shape box = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();

    gp_Pnt center(7.50, 7.50, 2.0);
    gp_Dir axisDirection(0.0, 0.0, 1.0);
    gp_Ax2 axis(center, axisDirection);
    TopoDS_Shape cylinder = BRepPrimAPI_MakeCylinder(axis, 2.0, 10.0).Shape();
    TopoDS_Shape shape = BRepAlgoAPI_Cut(box, cylinder).Shape();

    // Convert CAD shape to geometry + topology
    Readers::TopoDS_ShapeConverter converter(shape);

    // Discretization settings: 8 segments per edge for a reasonably resolved edge mesh
    Geometry3D::DiscretizationSettings3D settings(8, 2);

    // S1 pipeline: TwinTableGenerator → BoundaryDiscretizer3D → FacetTriangulationManager
    Meshing::SurfaceMeshingContext3D context(converter.getGeometryCollection(),
                                             converter.getTopology(),
                                             settings);

    const auto& discResult = context.getDiscretizationResult();
    std::cout << "Points:         " << discResult.points.size() << "\n";
    std::cout << "Topology edges: " << discResult.edgeIdToPointIndicesMap.size() << "\n";
    std::cout << "Faces:          " << context.getFacetTriangulationManager().size() << "\n";

    // Export discretized edges for ParaView inspection
    // Color by EdgeID to verify each topology edge and check shared-edge alignment
    Export::VtkExporter exporter;
    exporter.writeEdgeMesh(discResult, "SurfaceMeshEdges.vtu");
    std::cout << "Exported edge mesh to SurfaceMeshEdges.vtu\n";
    std::cout << "Open in ParaView and color by EdgeID.\n";

    return 0;
}
