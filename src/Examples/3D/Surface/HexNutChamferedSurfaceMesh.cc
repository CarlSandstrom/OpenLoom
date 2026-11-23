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

#include "ExampleShapes3D.h"
#include "Common/Logging.h"
#include "Export/TsvExporter.h"
#include "Export/VtkExporter.h"
#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/General/BoundaryDiscretizer3D.h"
#include "Meshing/Core/3D/Surface/SurfaceMesher3D.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"
#include "Readers/OpenCascade/TopoDS_ShapeConverter.h"

#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRep_Tool.hxx>
#include <Geom_Curve.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>

#include <cmath>
#include <iostream>
#include <numbers>
#include <optional>

namespace
{

constexpr double CHAMFER_SIZE = 0.5; // bore chamfer leg length

// The plain M8 hex nut with both bore openings chamfered.
TopoDS_Shape buildChamferedHexNut()
{
    TopoDS_Shape nutBody = ExampleShapes3D::buildHexNut();

    // Chamfer both bore openings so the chamfer's outer circle lands exactly at
    // z = 0 / z = HEIGHT: the bore edge at each opening is the circle where
    // the curve's end points sit at radius BORE_RADIUS and share a z value.
    BRepFilletAPI_MakeChamfer chamferMaker(nutBody);
    for (TopExp_Explorer edgeExplorer(nutBody, TopAbs_EDGE); edgeExplorer.More(); edgeExplorer.Next())
    {
        TopoDS_Edge edge = TopoDS::Edge(edgeExplorer.Current());
        double firstParameter = 0.0;
        double lastParameter = 0.0;
        Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, firstParameter, lastParameter);
        if (curve.IsNull())
            continue;

        gp_Pnt point0 = curve->Value(firstParameter);
        gp_Pnt point1 = curve->Value(lastParameter);
        bool onBoreCircle = std::abs(std::hypot(point0.X(), point0.Y()) - ExampleShapes3D::HEX_NUT_BORE_RADIUS) < 1e-6 &&
                            std::abs(std::hypot(point1.X(), point1.Y()) - ExampleShapes3D::HEX_NUT_BORE_RADIUS) < 1e-6;
        bool sharedFlatFace = std::abs(point0.Z() - point1.Z()) < 1e-6 &&
                              (std::abs(point0.Z()) < 1e-6 || std::abs(point0.Z() - ExampleShapes3D::HEX_NUT_HEIGHT) < 1e-6);
        if (onBoreCircle && sharedFlatFace)
            chamferMaker.Add(CHAMFER_SIZE, edge);
    }

    return chamferMaker.Shape();
}

} // namespace

int main()
{
    Common::initializeLogging();

    TopoDS_Shape shape = buildChamferedHexNut();
    Readers::TopoDS_ShapeConverter converter(shape);

    Geometry3D::DiscretizationSettings3D discretizationSettings(std::nullopt, std::numbers::pi / 8.0, 2);

    auto discretizationResult =
        Meshing::BoundaryDiscretizer3D::discretize(converter.getGeometryCollection(),
                                                   converter.getTopology(),
                                                   discretizationSettings);

    std::cout << "Points:         " << discretizationResult->points.size() << "\n";
    std::cout << "Topology edges: " << discretizationResult->edgeIdToPointIndicesMap.size() << "\n";

    Export::VtkExporter exporter;
    exporter.writeEdgeMesh(*discretizationResult, "HexNutChamferedEdges.vtu");
    Export::TsvExporter::writeDiscretization(*discretizationResult, "HexNutChamferedEdges");
    std::cout << "Exported edge mesh to HexNutChamferedEdges.vtu (color by EdgeID)\n";

    Meshing::SurfaceMesher3D mesher(converter.getGeometryCollection(),
                                    converter.getTopology(),
                                    discretizationSettings,
                                    Meshing::SurfaceMesh3DQualitySettings{});

    mesher.mesh();
    auto& surfaceMesh = *mesher.getMesh();

    std::cout << "HexNutChamfered: " << surfaceMesh.nodes.size() << " nodes, "
              << surfaceMesh.triangles.size() << " triangles\n";

    exporter.writeSurfaceMesh(surfaceMesh, "HexNutChamfered.vtu");
    Export::TsvExporter::writeSurfaceMesh(surfaceMesh, "HexNutChamfered");
    std::cout << "Exported refined mesh to HexNutChamfered.vtu (color by SurfaceID)\n";

    return 0;
}
