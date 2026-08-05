/**
 * @file HexNutChamferedSurfaceMesh.cc
 * @brief Reproduction case for OPE-173: RCDT crash on a chamfered M8 hex nut.
 *
 * Same base geometry as HexNutSurfaceMesh (OPE-123), but with both bore
 * openings chamfered. The chamfer's outer circle lands exactly at z = 0 and
 * z = HEIGHT, alongside the hex's own bottom/top edges -- many coplanar
 * boundary points on the same flat face plane. That combination drives
 * MeshOperations3D::growCavityThroughCoplanarFaces to grow the Bowyer-Watson
 * cavity into a non-simply-connected region during RCDT refinement, which
 * corrupts the tetrahedralization and eventually throws INVALID_TOPOLOGY (see
 * OPE-173 for the full root-cause writeup).
 *
 * The non-chamfered hex nut (HexNutSurfaceMesh) does not trigger this path --
 * see the ticket for why the extra chamfer circle is the critical ingredient.
 *
 * This is a stress test, not a fast unit test: reaching the failure currently
 * takes several hundred Steiner point insertions in a Debug build.
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
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRep_Tool.hxx>
#include <Geom_Curve.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
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
constexpr double ACROSS_FLATS = 13.0; // flat-to-flat width
constexpr double HEIGHT = 6.5;        // nut height
constexpr double BORE_RADIUS = 4.0;   // M8 bore (8 mm diameter)
constexpr double CHAMFER_SIZE = 0.5;  // bore chamfer leg length

TopoDS_Shape buildChamferedHexNut()
{
    // The circumscribed radius (vertex-to-center) for a regular hexagon with
    // flat-to-flat distance ACROSS_FLATS: r = (ACROSS_FLATS / 2) / cos(30°) = ACROSS_FLATS / sqrt(3).
    const double circumscribedRadius = ACROSS_FLATS / std::sqrt(3.0);

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

    const gp_Ax2 boreAxis(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0));
    const TopoDS_Shape bore = BRepPrimAPI_MakeCylinder(boreAxis, BORE_RADIUS, HEIGHT).Shape();
    TopoDS_Shape nutBody = BRepAlgoAPI_Cut(hexPrism, bore).Shape();

    // Chamfer both bore openings so the chamfer's outer circle lands exactly at
    // z = 0 / z = HEIGHT: the bore edge at each opening is the circle where
    // curve->Value(first/last) sit at radius BORE_RADIUS and share a z value.
    BRepFilletAPI_MakeChamfer chamferMaker(nutBody);
    for (TopExp_Explorer edgeExplorer(nutBody, TopAbs_EDGE); edgeExplorer.More(); edgeExplorer.Next())
    {
        const TopoDS_Edge edge = TopoDS::Edge(edgeExplorer.Current());
        double first = 0.0;
        double last = 0.0;
        const Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);
        if (curve.IsNull())
            continue;

        const gp_Pnt p0 = curve->Value(first);
        const gp_Pnt p1 = curve->Value(last);
        const bool onBoreCircle = std::abs(std::hypot(p0.X(), p0.Y()) - BORE_RADIUS) < 1e-6 &&
                                   std::abs(std::hypot(p1.X(), p1.Y()) - BORE_RADIUS) < 1e-6;
        const bool sharedFlatFace = std::abs(p0.Z() - p1.Z()) < 1e-6 &&
                                     (std::abs(p0.Z()) < 1e-6 || std::abs(p0.Z() - HEIGHT) < 1e-6);
        if (onBoreCircle && sharedFlatFace)
        {
            chamferMaker.Add(CHAMFER_SIZE, edge);
        }
    }

    return chamferMaker.Shape();
}

} // namespace

int main()
{
    Common::initLogging();

    const TopoDS_Shape shape = buildChamferedHexNut();
    Readers::TopoDS_ShapeConverter converter(shape);

    Geometry3D::DiscretizationSettings3D discSettings(std::nullopt, std::numbers::pi / 8.0, 2);

    Meshing::BoundaryDiscretizer3D discretizer(converter.getGeometryCollection(),
                                               converter.getTopology(),
                                               discSettings);
    discretizer.discretize();
    auto discResult = discretizer.releaseDiscretizationResult();

    std::cout << "Points:         " << discResult->points.size() << "\n";
    std::cout << "Topology edges: " << discResult->edgeIdToPointIndicesMap.size() << "\n";

    Export::VtkExporter exporter;
    exporter.writeEdgeMesh(*discResult, "HexNutChamferedEdges.vtu");
    std::cout << "Exported edge mesh to HexNutChamferedEdges.vtu (color by EdgeID)\n";

    Meshing::SurfaceMesher3D mesher(converter.getGeometryCollection(),
                                    converter.getTopology(),
                                    discSettings,
                                    Meshing::SurfaceMesh3DQualitySettings{});

    auto surfaceMesh = mesher.mesh();

    std::cout << "HexNutChamfered: " << surfaceMesh.nodes.size() << " nodes, "
              << surfaceMesh.triangles.size() << " triangles\n";

    exporter.writeSurfaceMesh(surfaceMesh, "HexNutChamfered.vtu");
    std::cout << "Exported refined mesh to HexNutChamfered.vtu (color by SurfaceID)\n";

    return 0;
}
