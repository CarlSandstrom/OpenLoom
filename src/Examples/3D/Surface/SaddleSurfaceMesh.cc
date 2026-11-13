/**
 * @file SaddleSurfaceMesh.cc
 * @brief Stress test: surface mesh of a hyperbolic paraboloid (z = x²−y²).
 *
 * The saddle surface has strictly negative Gaussian curvature at every point —
 * it curves upward in one principal direction (x) and downward in the other (y).
 * Gaussian curvature K = −4 / (1 + 4x² + 4y²)² is spatially varying: it peaks
 * in magnitude at the origin (K = −4) and decays toward the corners (|K| → 0).
 * This forces RCDT's chord-deviation refinement to densify the mesh near the
 * origin and coarsen it toward the corners — the primary mechanism under test.
 *
 * The top surface (z = x²−y²) is a biquadratic Bezier face. Its control-point
 * z-values come from the tensor-product Bernstein decomposition:
 *   z(u,v) = L²(2u−1)² − L²(2v−1)²,  x = L(2u−1),  y = L(2v−1)
 *   cᵢⱼ = αᵢ − βⱼ  with  α = β = [L², −L², L²]
 *
 * RCDT meshes the boundary of a closed solid, so a single open face cannot be
 * meshed: its boundary edges would each carry only one restricted triangle.
 * The solid is therefore closed by adding a flat bottom and four ruled side
 * faces — each a linear-in-depth Bezier surface whose top row matches the
 * saddle's parabolic boundary arc exactly.
 *
 * Topology:
 *   - 6 faces  : saddle top, flat bottom, 4 ruled sides
 *   - 12 edges : 4 parabolic top arcs, 4 vertical straight edges, 4 straight bottom edges
 *   - 8 corners: 4 top at (±L, ±L, 0),  4 bottom at (±L, ±L, Z_BOTTOM)
 *
 * Exports (each also written as .tsv tables):
 *   - SaddleSurfaceMeshEdges.vtu : discretized boundary edges (color by EdgeID)
 *   - SaddleSurfaceMesh.vtu      : final refined surface mesh (color by SurfaceID)
 */

#include "Common/Logging.h"
#include "Export/TsvExporter.h"
#include "Export/VtkExporter.h"
#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Connectivity/EdgeKey.h"
#include "Meshing/Core/3D/General/BoundaryDiscretizer3D.h"
#include "Meshing/Core/3D/Surface/SurfaceMesher3D.h"
#include "Meshing/Data/3D/SurfaceMesh3D.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"
#include "Readers/OpenCascade/TopoDS_ShapeConverter.h"

#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <Geom_BezierSurface.hxx>
#include <Precision.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Shell.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>

#include <array>
#include <iostream>
#include <map>
#include <numbers>

namespace
{

constexpr double LENGTH = 2.0;    // half-extent of the saddle domain in x and y
constexpr double Z_BOTTOM = -5.0; // flat bottom, below the lowest saddle point (z = -L² = -4)

// Control grid of the saddle z = x²−y² as a biquadratic Bezier surface.
// Index i runs along x and j along y; cᵢⱼ = αᵢ − βⱼ with α = β = [L², −L², L²].
TColgp_Array2OfPnt makeSaddlePoles()
{
    double squaredLength = LENGTH * LENGTH;

    TColgp_Array2OfPnt poles(1, 3, 1, 3);
    poles.SetValue(1, 1, gp_Pnt(-LENGTH, -LENGTH, 0.0));
    poles.SetValue(1, 2, gp_Pnt(-LENGTH, 0.0, 2.0 * squaredLength));
    poles.SetValue(1, 3, gp_Pnt(-LENGTH, LENGTH, 0.0));
    poles.SetValue(2, 1, gp_Pnt(0.0, -LENGTH, -2.0 * squaredLength));
    poles.SetValue(2, 2, gp_Pnt(0.0, 0.0, 0.0));
    poles.SetValue(2, 3, gp_Pnt(0.0, LENGTH, -2.0 * squaredLength));
    poles.SetValue(3, 1, gp_Pnt(LENGTH, -LENGTH, 0.0));
    poles.SetValue(3, 2, gp_Pnt(LENGTH, 0.0, 2.0 * squaredLength));
    poles.SetValue(3, 3, gp_Pnt(LENGTH, LENGTH, 0.0));
    return poles;
}

TopoDS_Face makeBezierFace(const TColgp_Array2OfPnt& poles)
{
    return BRepBuilderAPI_MakeFace(new Geom_BezierSurface(poles), Precision::Confusion()).Face();
}

TopoDS_Face makeBottomFace()
{
    gp_Pln plane(gp_Pnt(0.0, 0.0, Z_BOTTOM), gp_Dir(0.0, 0.0, 1.0));
    return BRepBuilderAPI_MakeFace(plane, -LENGTH, LENGTH, -LENGTH, LENGTH).Face();
}

// A side face: a Bezier surface linear in U (depth) and quadratic in V (along
// the arc). Its top row is the saddle's boundary arc pole-for-pole, so the
// shared edge is geometrically identical and sewing closes it without a gap;
// its bottom row is the same arc dropped to z = Z_BOTTOM.
TopoDS_Face makeRuledSideFace(const std::array<gp_Pnt, 3>& arcPoles)
{
    TColgp_Array2OfPnt poles(1, 2, 1, 3);
    for (int column = 1; column <= 3; ++column)
    {
        auto& arcPole = arcPoles[column - 1];
        poles.SetValue(1, column, arcPole);
        poles.SetValue(2, column, gp_Pnt(arcPole.X(), arcPole.Y(), Z_BOTTOM));
    }
    return makeBezierFace(poles);
}

// Builds the saddle solid: the hyperbolic paraboloid z = x²−y² as the top face,
// a flat bottom, and four ruled side faces, sewn into a closed shell.
TopoDS_Shape buildSaddleSolid()
{
    auto saddlePoles = makeSaddlePoles();

    BRepBuilderAPI_Sewing sewing;
    sewing.Add(makeBezierFace(saddlePoles));
    sewing.Add(makeBottomFace());
    sewing.Add(makeRuledSideFace({saddlePoles(1, 1), saddlePoles(1, 2), saddlePoles(1, 3)})); // x = −L
    sewing.Add(makeRuledSideFace({saddlePoles(3, 1), saddlePoles(3, 2), saddlePoles(3, 3)})); // x = +L
    sewing.Add(makeRuledSideFace({saddlePoles(1, 1), saddlePoles(2, 1), saddlePoles(3, 1)})); // y = −L
    sewing.Add(makeRuledSideFace({saddlePoles(1, 3), saddlePoles(2, 3), saddlePoles(3, 3)})); // y = +L
    sewing.Perform();

    // Sewing only produces a closed TopoDS_Shell -- wrap it into an actual
    // TopoDS_Solid so downstream consumers that need genuine solid topology
    // (e.g. an inside/outside classifier querying the CAD model directly)
    // have one to query, not just a shell that happens to be watertight.
    return BRepBuilderAPI_MakeSolid(TopoDS::Shell(sewing.SewedShape())).Solid();
}

struct NonManifoldEdgeCounts
{
    int holeEdgeCount = 0;
    int excessEdgeCount = 0;
};

// Edge multiplicity across the restricted face set. An edge shared by
// exactly 2 triangles is a normal closed-manifold interior edge -- that
// includes the creases, where the two triangles come from different CAD
// surfaces.
//
// This sees holes and excess faces only. It is blind to the third defect
// kind, an edge carrying the expected NUMBER of faces but on the wrong
// surfaces, so it under-reports -- RestrictedFaceAudit, which classifies
// all three, is the authority for any number quoted from this model.
NonManifoldEdgeCounts countNonManifoldEdges(const Meshing::SurfaceMesh3D& surfaceMesh)
{
    std::map<Meshing::EdgeKey, int> edgeMultiplicity;
    for (auto& triangle : surfaceMesh.triangles)
    {
        for (size_t cornerIndex = 0; cornerIndex < 3; ++cornerIndex)
            ++edgeMultiplicity[Meshing::EdgeKey(triangle[cornerIndex], triangle[(cornerIndex + 1) % 3])];
    }

    NonManifoldEdgeCounts counts;
    for (auto& [edge, faceCount] : edgeMultiplicity)
    {
        if (faceCount < 2)
            ++counts.holeEdgeCount;
        else if (faceCount > 2)
            ++counts.excessEdgeCount;
    }
    return counts;
}

} // namespace

int main()
{
    Common::initializeLogging();

    TopoDS_Shape shape = buildSaddleSolid();
    Readers::TopoDS_ShapeConverter converter(shape);

    // π/8 (22.5°) angle threshold: the parabolic arcs on the four sides are
    // curved enough to be densely sampled at this threshold, giving the interior
    // refiner sufficient boundary constraint nodes along each curved edge.
    Geometry3D::DiscretizationSettings3D discretizationSettings(std::nullopt, std::numbers::pi / 8.0, 2);

    auto discretizationResult =
        Meshing::BoundaryDiscretizer3D::discretize(converter.getGeometryCollection(),
                                                   converter.getTopology(),
                                                   discretizationSettings);

    std::cout << "Points:         " << discretizationResult->points.size() << "\n";
    std::cout << "Topology edges: " << discretizationResult->edgeIdToPointIndicesMap.size() << "\n";

    Export::VtkExporter exporter;
    exporter.writeEdgeMesh(*discretizationResult, "SaddleSurfaceMeshEdges.vtu");
    Export::TsvExporter::writeDiscretization(*discretizationResult, "SaddleSurfaceMeshEdges");
    std::cout << "Exported edge mesh to SaddleSurfaceMeshEdges.vtu\n";

    // Default quality settings: minAngleDegrees = 30, chordDeviationTolerance = 0.1.
    //
    // The chord-deviation criterion is the mechanism under test: facets too
    // far from the CAD surface are refined, producing locally finer meshes
    // where |K| is large (near origin) and coarser where |K| is small (near
    // corners).
    //
    // The saddle solid's four top corners have a ~20° interior angle between
    // the two parabolic boundary arcs that meet there. The protecting balls
    // (ProtectingBallPlacer) keep refinement out of those corners, and the
    // criteria are waived for facets touching a ball, as in CGAL Mesh_3.
    //
    // The restricted surface is closed with no excess faces, here and at 218
    // boundary points (angle threshold pi/64).

    Meshing::SurfaceMesh3DQualitySettings quality;

    Meshing::SurfaceMesher3D mesher(converter.getGeometryCollection(),
                                    converter.getTopology(),
                                    discretizationSettings,
                                    quality);
    mesher.mesh();
    auto& surfaceMesh = *mesher.getMesh();

    std::cout << "SurfaceMesh3D: " << surfaceMesh.nodes.size() << " nodes, "
              << surfaceMesh.triangles.size() << " triangles\n";

    exporter.writeSurfaceMesh(surfaceMesh, "SaddleSurfaceMesh.vtu");
    Export::TsvExporter::writeSurfaceMesh(surfaceMesh, "SaddleSurfaceMesh");
    std::cout << "Exported refined mesh to SaddleSurfaceMesh.vtu\n";

    auto nonManifoldEdges = countNonManifoldEdges(surfaceMesh);
    std::cout << "Non-manifold edges: " << nonManifoldEdges.holeEdgeCount << " holes, "
              << nonManifoldEdges.excessEdgeCount
              << " excess (surface mismatches not visible here -- see the runPipeline log)\n";

    return 0;
}
