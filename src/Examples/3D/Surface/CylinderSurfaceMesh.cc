/**
 * @file CylinderSurfaceMesh.cc
 * @brief Simple cylinder surface mesh example with a seam edge.
 *
 * A cylinder is the minimal CAD shape that contains a seam: the lateral face
 * is a closed surface whose parametric domain is stitched along a vertical
 * seam edge.  This makes it the ideal minimal test for seam handling in the
 * surface meshing pipeline.
 *
 * Topology:
 *   - 3 faces   : bottom cap, top cap, lateral (cylindrical) face
 *   - 1 seam    : vertical edge on the lateral face
 *   - 2 circles : top and bottom boundary circles
 *
 * Exports:
 *   - CylinderSurfaceMeshEdges.vtu : discretized boundary edges (color by EdgeID)
 *   - CylinderSurfaceMesh3D.vtu    : full surface triangulation (color by SurfaceID)
 */

#include "../Readers/OpenCascade/TopoDS_ShapeConverter.h"
#include "Export/VtkExporter.h"
#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/General/DiscretizationResult3D.h"
#include "Meshing/Core/3D/General/FacetTriangulationManager.h"
#include "Meshing/Core/3D/Surface/SurfaceMeshingContext3D.h"
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <iostream>
#include <numbers>

int main()
{
    // Plain cylinder: radius 3, height 8, centred at the origin along Z
    gp_Pnt origin(0.0, 0.0, 0.0);
    gp_Dir axisZ(0.0, 0.0, 1.0);
    gp_Ax2 axis(origin, axisZ);
    TopoDS_Shape shape = BRepPrimAPI_MakeCylinder(axis, 3.0, 8.0).Shape();

    // Convert CAD shape to geometry + topology
    Readers::TopoDS_ShapeConverter converter(shape);

    // Angle-based discretization: insert edge points wherever the tangent
    // direction changes by more than π/8 (22.5°).
    Geometry3D::DiscretizationSettings3D settings(std::nullopt, std::numbers::pi / 8.0, 2);

    // S1 pipeline: TwinTableGenerator → BoundaryDiscretizer3D → FacetTriangulationManager
    Meshing::SurfaceMeshingContext3D context(converter.getGeometryCollection(),
                                             converter.getTopology(),
                                             settings);

    const auto& discResult = context.getDiscretizationResult();

    std::cout << "Points:         " << discResult.points.size() << "\n";
    std::cout << "Topology edges: " << discResult.edgeIdToPointIndicesMap.size() << "\n";
    std::cout << "Faces:          " << context.getFacetTriangulationManager().size() << "\n";

    Export::VtkExporter exporter;

    exporter.writeEdgeMesh(discResult, "CylinderSurfaceMeshEdges.vtu");
    std::cout << "Exported edge mesh to CylinderSurfaceMeshEdges.vtu (color by EdgeID)\n";

    // Angle-quality pass only (chord deviation disabled).
    // Uncomment the second argument to also enforce geometric fidelity:
    //   context.refineSurfaces(2.0, 30.0, 50000, 0.05);
    // 0.05 means every triangle must approximate the CAD surface to within 0.05
    // model units at its centroid and edge midpoints.
    context.refineSurfaces();

    const auto subfacets = context.getFacetTriangulationManager().getAllSubfacets();
    std::cout << "Subfacets: " << subfacets.size() << "\n";

    auto meshData = context.getSurfaceMesh3D();
    std::cout << "MeshData3D: " << meshData.getNodeCount() << " nodes, " << meshData.getElementCount() << " triangles\n";

    exporter.writeSurfaceMesh(meshData, subfacets, "CylinderSurfaceMesh3D.vtu");
    std::cout << "Exported surface mesh to CylinderSurfaceMesh3D.vtu (color by SurfaceID)\n";

    return 0;
}
