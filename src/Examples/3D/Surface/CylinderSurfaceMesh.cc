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
#include "Common/Logging.h"
#include "Export/VtkExporter.h"
#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"
#include "Meshing/Core/3D/General/DiscretizationResult3D.h"
#include "Meshing/Core/3D/General/FacetTriangulationManager.h"
#include "Meshing/Core/3D/Surface/SurfaceMesher3D.h"
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
    Common::initLogging();

    // Plain cylinder: radius 3, height 8, centred at the origin along Z
    gp_Pnt origin(0.0, 0.0, 0.0);
    gp_Dir axisZ(0.0, 0.0, 1.0);
    gp_Ax2 axis(origin, axisZ);
    TopoDS_Shape shape = BRepPrimAPI_MakeCylinder(axis, 3.0, 8.0).Shape();

    // Convert CAD shape to geometry + topology
    Readers::TopoDS_ShapeConverter converter(shape);

    // Angle-based discretization: insert edge points wherever the tangent
    // direction changes by more than π/8 (22.5°).
    Geometry3D::DiscretizationSettings3D discSettings(std::nullopt, std::numbers::pi / 8.0, 2);

    // Export the discretized boundary edges before refinement.
    // We need the raw context for this pre-refinement export.
    Meshing::SurfaceMeshingContext3D context(converter.getGeometryCollection(),
                                             converter.getTopology(),
                                             discSettings,
                                             Meshing::SurfaceMesh3DQualitySettings{});

    const auto& discResult = context.getDiscretizationResult();
    std::cout << "Points:         " << discResult.points.size() << "\n";
    std::cout << "Topology edges: " << discResult.edgeIdToPointIndicesMap.size() << "\n";
    std::cout << "Faces:          " << context.getFacetTriangulationManager().size() << "\n";

    Export::VtkExporter exporter;
    exporter.writeEdgeMesh(discResult, "CylinderSurfaceMeshEdges.vtu");
    std::cout << "Exported edge mesh to CylinderSurfaceMeshEdges.vtu (color by EdgeID)\n";

    // Run the full S1–S3 pipeline via SurfaceMesher3D and export the result.
    Meshing::SurfaceMesher3D mesher(converter.getGeometryCollection(),
                                    converter.getTopology(),
                                    discSettings,
                                    Meshing::SurfaceMesh3DQualitySettings{});

    auto surfaceMesh = mesher.mesh();

    std::cout << "SurfaceMesh3D: " << surfaceMesh.nodes.size() << " nodes, "
              << surfaceMesh.triangles.size() << " triangles\n";

    exporter.writeSurfaceMesh(surfaceMesh, "CylinderSurfaceMesh3D.vtu");
    std::cout << "Exported surface mesh to CylinderSurfaceMesh3D.vtu (color by SurfaceID)\n";

    return 0;
}
