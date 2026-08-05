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
 * The cylindrical bore is the only periodic surface, so SurfaceMesher3D's Auto
 * dispatch routes the whole shape through AmbientRCDT (not the legacy UV-space
 * path), exercising SeamCollection handling on a multi-face topology.
 *
 * Exports:
 *   - HexNutEdges.vtu  : discretized boundary edges (color by EdgeID)
 *   - HexNut.vtu       : refined surface mesh (color by SurfaceID)
 */

#include "Common/Logging.h"
#include "Export/VtkExporter.h"
#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/General/BoundaryDiscretizer3D.h"
#include "Meshing/Core/3D/Surface/SurfaceMesher3D.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"
#include "Readers/OpenCascade/TopoDS_ShapeConverter.h"

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <cmath>
#include <iostream>
#include <numbers>

namespace
{

// M8 hex nut geometry (metric standard, all dimensions in mm)
constexpr double ACROSS_FLATS = 13.0;  // flat-to-flat width
constexpr double HEIGHT = 6.5;          // nut height
constexpr double BORE_RADIUS = 4.0;    // M8 bore (8 mm diameter)

TopoDS_Shape buildHexNut()
{
    // The circumscribed radius (vertex-to-center) for a regular hexagon with
    // flat-to-flat distance ACROSS_FLATS: r = (ACROSS_FLATS / 2) / cos(30°) = ACROSS_FLATS / sqrt(3).
    const double circumscribedRadius = ACROSS_FLATS / std::sqrt(3.0);

    // Hexagonal base: vertices at 30°, 90°, 150°, 210°, 270°, 330° place one flat
    // face toward +X (i.e., the right-hand flat is vertical, parallel to Y).
    BRepBuilderAPI_MakePolygon hexPolygon;
    for (int i = 0; i < 6; ++i)
    {
        const double angle = (2.0 * i + 1.0) * std::numbers::pi / 6.0;
        hexPolygon.Add(gp_Pnt(circumscribedRadius * std::cos(angle),
                              circumscribedRadius * std::sin(angle),
                              0.0));
    }
    hexPolygon.Close();

    const TopoDS_Face hexFace = BRepBuilderAPI_MakeFace(hexPolygon.Wire()).Face();
    TopoDS_Shape hexPrism = BRepPrimAPI_MakePrism(hexFace, gp_Vec(0.0, 0.0, HEIGHT)).Shape();

    // Main bore: full-height cylinder, radius = BORE_RADIUS.
    const gp_Ax2 boreAxis(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0));
    const TopoDS_Shape bore = BRepPrimAPI_MakeCylinder(boreAxis, BORE_RADIUS, HEIGHT).Shape();
    TopoDS_Shape nutBody = BRepAlgoAPI_Cut(hexPrism, bore).Shape();

    return nutBody;
}

} // namespace

int main()
{
    Common::initLogging();

    const TopoDS_Shape shape = buildHexNut();
    Readers::TopoDS_ShapeConverter converter(shape);

    Geometry3D::DiscretizationSettings3D discSettings(std::nullopt, std::numbers::pi / 8.0, 2);

    // Export the discretized boundary edges for inspection.
    Meshing::BoundaryDiscretizer3D discretizer(converter.getGeometryCollection(),
                                               converter.getTopology(),
                                               discSettings);
    discretizer.discretize();
    auto discResult = discretizer.releaseDiscretizationResult();

    std::cout << "Points:         " << discResult->points.size() << "\n";
    std::cout << "Topology edges: " << discResult->edgeIdToPointIndicesMap.size() << "\n";

    Export::VtkExporter exporter;
    exporter.writeEdgeMesh(*discResult, "HexNutEdges.vtu");
    std::cout << "Exported edge mesh to HexNutEdges.vtu (color by EdgeID)\n";

    // Auto dispatch routes this shape through AmbientRCDT because the cylindrical
    // bore is a periodic (seam) surface — the SeamCollection is non-empty.
    Meshing::SurfaceMesher3D mesher(converter.getGeometryCollection(),
                                    converter.getTopology(),
                                    discSettings,
                                    Meshing::SurfaceMesh3DQualitySettings{});

    auto surfaceMesh = mesher.mesh();

    std::cout << "HexNut: " << surfaceMesh.nodes.size() << " nodes, "
              << surfaceMesh.triangles.size() << " triangles\n";

    exporter.writeSurfaceMesh(surfaceMesh, "HexNut.vtu");
    std::cout << "Exported refined mesh to HexNut.vtu (color by SurfaceID)\n";

    return 0;
}
