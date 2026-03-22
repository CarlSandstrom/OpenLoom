/**
 * @file ThinFinSurfaceMesh.cc
 * @brief Stress test: surface mesh of a thin fin / blade shape (100×10×0.5 box).
 *
 * The two 100×0.5 long side faces have a 200:1 aspect ratio and flat (planar)
 * geometry. Nearly degenerate surface normals and high aspect ratio test
 * surface sampling and triangle quality in thin patches.
 *
 * Topology:
 *   - 6 faces : two 100×10 caps, two 100×0.5 thin sides, two 10×0.5 ends
 *   - 12 edges, 8 corners (rectangular box)
 *
 * Exports:
 *   - ThinFinSurfaceMeshEdges.vtu   : discretized boundary edges (color by EdgeID)
 *   - ThinFinSurfaceMeshInitial.vtu : initial triangulation before refinement
 *   - ThinFinSurfaceMeshRefined.vtu : final refined surface mesh (color by SurfaceID)
 */

#include "Common/Logging.h"
#include "Export/VtkExporter.h"
#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/General/DiscretizationResult3D.h"
#include "Meshing/Core/3D/General/FacetTriangulationManager.h"
#include "Meshing/Core/3D/Surface/SurfaceMesh3DQualitySettings.h"
#include "Meshing/Core/3D/Surface/SurfaceMeshingContext3D.h"
#include "Readers/OpenCascade/TopoDS_ShapeConverter.h"
#include <BRepPrimAPI_MakeBox.hxx>
#include <TopoDS_Shape.hxx>
#include <iostream>
#include <numbers>

int main()
{
    Common::initLogging();

    // 100×10×0.5 box: the two 100×0.5 faces are the thin fins under test.
    TopoDS_Shape shape = BRepPrimAPI_MakeBox(100.0, 10.0, 0.5).Shape();

    Readers::TopoDS_ShapeConverter converter(shape);

    // Angle-based discretization. The flat faces produce no tangent change
    // along edges, so edge sampling is minimal (just endpoints). Four interior
    // seed points per UV direction ensure the refiner has starting material on
    // each flat face.
    Geometry3D::DiscretizationSettings3D discSettings(std::nullopt, std::numbers::pi / 8.0, 4);

    // S1 runs at construction.
    Meshing::SurfaceMeshingContext3D context(converter.getGeometryCollection(),
                                             converter.getTopology(),
                                             discSettings,
                                             Meshing::SurfaceMesh3DQualitySettings{});

    const auto& discResult = context.getDiscretizationResult();
    const auto subfacets = context.getFacetTriangulationManager().getAllSubfacets();

    std::cout << "Points:         " << discResult.points.size() << "\n";
    std::cout << "Topology edges: " << discResult.edgeIdToPointIndicesMap.size() << "\n";
    std::cout << "Faces:          " << context.getFacetTriangulationManager().size() << "\n";

    Export::VtkExporter exporter;
    exporter.writeEdgeMesh(discResult, "ThinFinSurfaceMeshEdges.vtu");
    std::cout << "Exported edge mesh to ThinFinSurfaceMeshEdges.vtu\n";

    exporter.writeSurfaceMesh(discResult, subfacets, "ThinFinSurfaceMeshInitial.vtu");
    std::cout << "Exported initial mesh to ThinFinSurfaceMeshInitial.vtu\n";

    // S2–S3 refinement.
    context.refineSurfaces();
    auto surfaceMesh = context.buildSurfaceMesh();

    std::cout << "SurfaceMesh3D: " << surfaceMesh.nodes.size() << " nodes, "
              << surfaceMesh.triangles.size() << " triangles\n";

    exporter.writeSurfaceMesh(surfaceMesh, "ThinFinSurfaceMeshRefined.vtu");
    std::cout << "Exported refined mesh to ThinFinSurfaceMeshRefined.vtu\n";

    return 0;
}
