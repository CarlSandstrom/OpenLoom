/**
 * @file TorusSurfaceMesh.cc
 * @brief Surface mesh of a torus — a genus-1 closed surface with no boundary edges.
 *
 * OCC represents a full torus as a single toroidal face with two seam edges (one per
 * parametric direction). Both u and v are periodic, so the face adjacency graph is a
 * single node with self-loops in two independent directions. This exercises loop
 * detection and surface traversal in a topology that none of the simpler examples cover.
 *
 * Gaussian curvature is positive on the outer half and negative on the inner half, so
 * chord-deviation refinement (Phase 2) is enabled to maintain geometric fidelity across
 * both regions.
 *
 * Exports:
 *   - TorusSurfaceMeshEdges.vtu         : discretized seam edges (color by EdgeID)
 *   - TorusSurfaceMesh3D.vtu            : initial surface triangulation (color by SurfaceID)
 *   - TorusSurfaceMesh3D_Refined.vtu    : refined surface triangulation
 */

#include "../Readers/OpenCascade/TopoDS_ShapeConverter.h"
#include "Common/Logging.h"
#include "Export/VtkExporter.h"
#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/General/DiscretizationResult3D.h"
#include "Meshing/Core/3D/General/FacetTriangulationManager.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"
#include "Meshing/Core/3D/Surface/SurfaceMeshingContext3D.h"
#include <BRepPrimAPI_MakeTorus.hxx>
#include <TopoDS_Shape.hxx>
#include <iostream>
#include <numbers>

int main()
{
    Common::initLogging();

    // Major radius R1 = 5.0 (distance from torus centre to pipe centre)
    // Minor radius R2 = 1.5 (radius of the pipe / tube)
    // R1/R2 ≈ 3.3 keeps the inner equator well away from degenerate collapse.
    TopoDS_Shape shape = BRepPrimAPI_MakeTorus(5.0, 1.5).Shape();

    Readers::TopoDS_ShapeConverter converter(shape);

    Geometry3D::DiscretizationSettings3D settings(std::nullopt, std::numbers::pi / 8.0, 2);

    Meshing::SurfaceMesh3DQualitySettings quality;
    quality.chordDeviationTolerance = 0.15;

    Meshing::SurfaceMeshingContext3D context(converter.getGeometryCollection(),
                                             converter.getTopology(),
                                             settings,
                                             quality);

    const auto& discResult = context.getDiscretizationResult();
    const auto subfacets = context.getFacetTriangulationManager().getAllSubfacets();

    std::cout << "Points:         " << discResult.points.size() << "\n";
    std::cout << "Topology edges: " << discResult.edgeIdToPointIndicesMap.size() << "\n";
    std::cout << "Faces:          " << context.getFacetTriangulationManager().size() << "\n";
    std::cout << "Subfacets:      " << subfacets.size() << "\n";

    Export::VtkExporter exporter;

    exporter.writeEdgeMesh(discResult, "TorusSurfaceMeshEdges.vtu");
    std::cout << "Exported edge mesh to TorusSurfaceMeshEdges.vtu (color by EdgeID)\n";

    exporter.writeSurfaceMesh(discResult, subfacets, "TorusSurfaceMesh3D.vtu");
    std::cout << "Exported surface mesh to TorusSurfaceMesh3D.vtu (color by SurfaceID)\n";

    context.refineSurfaces();

    auto surfaceMesh = context.buildSurfaceMesh();
    std::cout << "Refined: " << surfaceMesh.nodes.size() << " nodes, "
              << surfaceMesh.triangles.size() << " triangles\n";

    exporter.writeSurfaceMesh(surfaceMesh, "TorusSurfaceMesh3D_Refined.vtu");
    std::cout << "Exported refined surface mesh to TorusSurfaceMesh3D_Refined.vtu\n";

    return 0;
}
