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
 * the chord-deviation criterion is loosened slightly to keep geometric fidelity across
 * both regions without over-refining.
 *
 * Exports:
 *   - TorusSurfaceMeshEdges.vtu : discretized seam edges (color by EdgeID)
 *   - TorusSurfaceMesh3D.vtu    : surface mesh (color by SurfaceID)
 */

#include "../Readers/OpenCascade/TopoDS_ShapeConverter.h"
#include "Common/Logging.h"
#include "Export/VtkExporter.h"
#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/General/BoundaryDiscretizer3D.h"
#include "Meshing/Core/3D/Surface/SurfaceMesher3D.h"
#include "Meshing/Data/3D/DiscretizationResult3D.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"
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

    const auto discResult = Meshing::BoundaryDiscretizer3D::discretize(converter.getGeometryCollection(),
                                                                       converter.getTopology(),
                                                                       settings);

    std::cout << "Points:         " << discResult->points.size() << "\n";
    std::cout << "Topology edges: " << discResult->edgeIdToPointIndicesMap.size() << "\n";
    std::cout << "Faces:          " << converter.getTopology().getAllSurfaceIds().size() << "\n";

    Export::VtkExporter exporter;

    exporter.writeEdgeMesh(*discResult, "TorusSurfaceMeshEdges.vtu");
    std::cout << "Exported edge mesh to TorusSurfaceMeshEdges.vtu (color by EdgeID)\n";

    Meshing::SurfaceMesher3D mesher(converter.getGeometryCollection(), converter.getTopology(), settings, quality);
    auto surfaceMesh = mesher.mesh();
    std::cout << "SurfaceMesh3D: " << surfaceMesh.nodes.size() << " nodes, "
              << surfaceMesh.triangles.size() << " triangles\n";

    exporter.writeSurfaceMesh(surfaceMesh, "TorusSurfaceMesh3D.vtu");
    std::cout << "Exported surface mesh to TorusSurfaceMesh3D.vtu (color by SurfaceID)\n";

    return 0;
}
