/**
 * @file HexNutSurfaceMesh.cc
 * @brief Stress test: surface mesh of an M8-sized hex nut with cylindrical bore.
 *
 * Models a metric M8 hex nut: a hexagonal prism with a cylindrical throughbore.
 * Exercises the RCDT surface mesher on a part with faces of significantly varying
 * size meeting at shared edges:
 *
 *   - 6 planar side faces (ACROSS_FLATS wide × HEIGHT tall)
 *   - 2 planar annular top/bottom faces (large hexagonal footprint minus bore opening)
 *   - 1 cylindrical bore face (periodic, diameter 8 mm × HEIGHT tall) -- seam surface
 *
 * The cylindrical bore is the only periodic surface, exercising SeamCollection
 * handling on a multi-face topology.
 *
 * Exports:
 *   - HexNutEdges.vtu  : discretized boundary edges (color by EdgeID)
 *   - HexNut.vtu       : refined surface mesh (color by SurfaceID)
 */

#include "Common/Logging.h"
#include "ExampleShapes3D.h"
#include "Export/TsvExporter.h"
#include "Export/VtkExporter.h"
#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/General/BoundaryDiscretizer3D.h"
#include "Meshing/Core/3D/Surface/SurfaceMesher3D.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"
#include "Readers/OpenCascade/TopoDS_ShapeConverter.h"

#include <TopoDS_Shape.hxx>

#include <iostream>
#include <numbers>
#include <optional>

int main()
{
    Common::initializeLogging();

    TopoDS_Shape shape = ExampleShapes3D::buildHexNut();
    Readers::TopoDS_ShapeConverter converter(shape);

    Geometry3D::DiscretizationSettings3D discretizationSettings(std::nullopt, std::numbers::pi / 8.0, 2);

    // Export the discretized boundary edges for inspection.
    auto discretizationResult =
        Meshing::BoundaryDiscretizer3D::discretize(converter.getGeometryCollection(),
                                                   converter.getTopology(),
                                                   discretizationSettings);

    std::cout << "Points:         " << discretizationResult->points.size() << "\n";
    std::cout << "Topology edges: " << discretizationResult->edgeIdToPointIndicesMap.size() << "\n";

    Export::VtkExporter exporter;
    exporter.writeEdgeMesh(*discretizationResult, "HexNutEdges.vtu");
    Export::TsvExporter::writeDiscretization(*discretizationResult, "HexNutEdges");
    std::cout << "Exported edge mesh to HexNutEdges.vtu (color by EdgeID)\n";

    Meshing::SurfaceMesher3D mesher(converter.getGeometryCollection(),
                                    converter.getTopology(),
                                    discretizationSettings,
                                    Meshing::SurfaceMesh3DQualitySettings{});

    mesher.mesh();
    auto& surfaceMesh = *mesher.getMesh();

    std::cout << "HexNut: " << surfaceMesh.nodes.size() << " nodes, "
              << surfaceMesh.triangles.size() << " triangles\n";

    exporter.writeSurfaceMesh(surfaceMesh, "HexNut.vtu");
    Export::TsvExporter::writeSurfaceMesh(surfaceMesh, "HexNut");
    std::cout << "Exported refined mesh to HexNut.vtu (color by SurfaceID)\n";

    return 0;
}
