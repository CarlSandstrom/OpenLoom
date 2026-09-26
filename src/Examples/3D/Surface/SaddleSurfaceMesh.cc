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
 * A single open face cannot be meshed by RCDT: boundary edges (shared by only
 * one restricted triangle) would be flagged as non-manifold defects by
 * Priority 4's repair loop, which can never converge for a genuinely open
 * boundary. The solid is therefore closed by adding a flat bottom and four
 * ruled side faces — each a linear-in-depth Bezier surface whose top row
 * matches the saddle's parabolic boundary arc exactly.
 *
 * Retargeted from the legacy per-face UV-space pipeline to SurfaceMesher3D's
 * AmbientRCDT strategy (OPE-122), since the UV-space path is being removed
 * (OPE-166) and curvature-adaptive sampling is mesher-agnostic.
 *
 * Topology:
 *   - 6 faces  : saddle top, flat bottom, 4 ruled sides
 *   - 12 edges : 4 parabolic top arcs, 4 vertical straight edges, 4 straight bottom edges
 *   - 8 corners: 4 top at (±L, ±L, 0),  4 bottom at (±L, ±L, Z_BOTTOM)
 *
 * Exports:
 *   - SaddleSurfaceMeshEdges.vtu : discretized boundary edges (color by EdgeID)
 *   - SaddleSurfaceMesh.vtu      : final refined surface mesh (color by SurfaceID)
 */

#include "Common/Logging.h"
#include "Export/TsvExporter.h"
#include "Export/VtkExporter.h"
#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/General/BoundaryDiscretizer3D.h"
#include "Meshing/Core/3D/Surface/SurfaceMesher3D.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"
#include "Readers/OpenCascade/TopoDS_ShapeConverter.h"

#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <Geom_BezierSurface.hxx>
#include <Precision.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Shell.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>

#include <iostream>
#include <map>
#include <numbers>

namespace
{

constexpr double L = 2.0;         // half-extent of the saddle domain in x and y
constexpr double Z_BOTTOM = -5.0; // flat bottom, below the lowest saddle point (z = -L² = -4)

// Builds the saddle solid: a closed shell consisting of the hyperbolic paraboloid
// z = x²−y² as the top face, a flat bottom, and four ruled side faces.
//
// The side faces are Bezier surfaces that are linear in U (depth direction) and
// quadratic in V (along the saddle's boundary arc). Their top row (u=0) matches
// the adjacent saddle boundary arc pole-for-pole, so the shared edges are
// geometrically identical and BRepBuilderAPI_Sewing closes them without gaps.
TopoDS_Shape buildSaddleSolid()
{
    const double squaredL = L * L;

    BRepBuilderAPI_Sewing sewing;

    // === Top face: saddle z = x²−y² (biquadratic Bezier, 3×3 control grid) ===
    // cᵢⱼ = αᵢ − βⱼ  where α = β = [L², −L², L²]
    {
        TColgp_Array2OfPnt poles(1, 3, 1, 3);
        poles.SetValue(1, 1, gp_Pnt(-L, -L, 0.0));
        poles.SetValue(1, 2, gp_Pnt(-L, 0.0, 2.0 * squaredL));
        poles.SetValue(1, 3, gp_Pnt(-L, L, 0.0));
        poles.SetValue(2, 1, gp_Pnt(0.0, -L, -2.0 * squaredL));
        poles.SetValue(2, 2, gp_Pnt(0.0, 0.0, 0.0));
        poles.SetValue(2, 3, gp_Pnt(0.0, L, -2.0 * squaredL));
        poles.SetValue(3, 1, gp_Pnt(L, -L, 0.0));
        poles.SetValue(3, 2, gp_Pnt(L, 0.0, 2.0 * squaredL));
        poles.SetValue(3, 3, gp_Pnt(L, L, 0.0));
        sewing.Add(BRepBuilderAPI_MakeFace(new Geom_BezierSurface(poles), Precision::Confusion()).Face());
    }

    // === Bottom face: flat rectangle at z = Z_BOTTOM ===
    {
        gp_Pln plane(gp_Pnt(0.0, 0.0, Z_BOTTOM), gp_Dir(0.0, 0.0, 1.0));
        sewing.Add(BRepBuilderAPI_MakeFace(plane, -L, L, -L, L).Face());
    }

    // === Left side (x = −L): ruled surface, top arc z = L²−y², bottom edge at z = Z_BOTTOM ===
    // Top row matches saddle column 1 (u=0) pole-for-pole.
    {
        TColgp_Array2OfPnt poles(1, 2, 1, 3);
        poles.SetValue(1, 1, gp_Pnt(-L, -L, 0.0));
        poles.SetValue(1, 2, gp_Pnt(-L, 0.0, 2.0 * squaredL));
        poles.SetValue(1, 3, gp_Pnt(-L, L, 0.0));
        poles.SetValue(2, 1, gp_Pnt(-L, -L, Z_BOTTOM));
        poles.SetValue(2, 2, gp_Pnt(-L, 0.0, Z_BOTTOM));
        poles.SetValue(2, 3, gp_Pnt(-L, L, Z_BOTTOM));
        sewing.Add(BRepBuilderAPI_MakeFace(new Geom_BezierSurface(poles), Precision::Confusion()).Face());
    }

    // === Right side (x = +L): ruled surface, top arc z = L²−y², bottom edge at z = Z_BOTTOM ===
    // Top row matches saddle column 3 (u=1) pole-for-pole.
    {
        TColgp_Array2OfPnt poles(1, 2, 1, 3);
        poles.SetValue(1, 1, gp_Pnt(L, -L, 0.0));
        poles.SetValue(1, 2, gp_Pnt(L, 0.0, 2.0 * squaredL));
        poles.SetValue(1, 3, gp_Pnt(L, L, 0.0));
        poles.SetValue(2, 1, gp_Pnt(L, -L, Z_BOTTOM));
        poles.SetValue(2, 2, gp_Pnt(L, 0.0, Z_BOTTOM));
        poles.SetValue(2, 3, gp_Pnt(L, L, Z_BOTTOM));
        sewing.Add(BRepBuilderAPI_MakeFace(new Geom_BezierSurface(poles), Precision::Confusion()).Face());
    }

    // === Front side (y = −L): ruled surface, top arc z = x²−L², bottom edge at z = Z_BOTTOM ===
    // Top row matches saddle row 1 (v=0) pole-for-pole.
    {
        TColgp_Array2OfPnt poles(1, 2, 1, 3);
        poles.SetValue(1, 1, gp_Pnt(-L, -L, 0.0));
        poles.SetValue(1, 2, gp_Pnt(0.0, -L, -2.0 * squaredL));
        poles.SetValue(1, 3, gp_Pnt(L, -L, 0.0));
        poles.SetValue(2, 1, gp_Pnt(-L, -L, Z_BOTTOM));
        poles.SetValue(2, 2, gp_Pnt(0.0, -L, Z_BOTTOM));
        poles.SetValue(2, 3, gp_Pnt(L, -L, Z_BOTTOM));
        sewing.Add(BRepBuilderAPI_MakeFace(new Geom_BezierSurface(poles), Precision::Confusion()).Face());
    }

    // === Back side (y = +L): ruled surface, top arc z = x²−L², bottom edge at z = Z_BOTTOM ===
    // Top row matches saddle row 3 (v=1) pole-for-pole.
    {
        TColgp_Array2OfPnt poles(1, 2, 1, 3);
        poles.SetValue(1, 1, gp_Pnt(-L, L, 0.0));
        poles.SetValue(1, 2, gp_Pnt(0.0, L, -2.0 * squaredL));
        poles.SetValue(1, 3, gp_Pnt(L, L, 0.0));
        poles.SetValue(2, 1, gp_Pnt(-L, L, Z_BOTTOM));
        poles.SetValue(2, 2, gp_Pnt(0.0, L, Z_BOTTOM));
        poles.SetValue(2, 3, gp_Pnt(L, L, Z_BOTTOM));
        sewing.Add(BRepBuilderAPI_MakeFace(new Geom_BezierSurface(poles), Precision::Confusion()).Face());
    }

    sewing.Perform();

    // Sewing only produces a closed TopoDS_Shell -- wrap it into an actual
    // TopoDS_Solid so downstream consumers that need genuine solid topology
    // (e.g. an inside/outside classifier querying the CAD model directly)
    // have one to query, not just a shell that happens to be watertight.
    const TopoDS_Shell shell = TopoDS::Shell(sewing.SewedShape());
    BRepBuilderAPI_MakeSolid solidMaker(shell);
    return solidMaker.Solid();
}

} // namespace

int main()
{
    Common::initLogging();

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

    // Default quality settings: circumradiusToShortestEdgeRatio = 1.0 (≡ min
    // angle ≥ 30°), chordDeviationTolerance = 0.1.
    //
    // The chord-deviation pass is the mechanism under test: triangles too far
    // from the CAD surface are split, producing locally finer meshes where |K|
    // is large (near origin) and coarser where |K| is small (near corners).
    //
    // The saddle solid's four top corners have a ~20° interior angle between
    // the two parabolic boundary arcs that meet there. Ruppert's termination
    // proof requires input angles ≥ 60°; at 20° the algorithm still
    // terminates correctly (the minimum edge length floor cuts off the
    // corner cascade) and now fully converges well within the iteration cap
    // (encroachment/bad-triangle tracking is incremental, not an O(n)
    // rescan).
    //
    // Known limitation: the finished mesh still carries a few non-manifold
    // edges in its restricted face set. Two of the three causes are closed:
    //
    //  - Crease ambiguity (OPE-176). classifyFace() has to decide which of two
    //    candidate surfaces a crease-straddling face belongs to, using a
    //    discrete tessellation of each surface as an exact crossing oracle;
    //    right at a shared boundary that is an inherently all-or-nothing call
    //    however fine the oracle's grid is. Weighted Delaunay refinement with
    //    protecting balls around every curve segment (see
    //    CurveProtectionScheme, RCDTMesher::buildInitial()) keeps creases as
    //    explicit, structurally protected features instead of repairing
    //    straddling faces after the fact, and cut the count from 863 to 17.
    //  - Same-surface over-acceptance (OPE-184). Doubled patches of restricted
    //    faces in the surface interior, now pruned as connected components by
    //    RestrictedFaceAudit::removeExcessFaces(): 17 to the residual below.
    //
    // What remains at this discretization is 2 edges carrying the expected
    // NUMBER of faces on the wrong surfaces -- a third defect kind, invariant
    // to the size floor and so far uninvestigated. Boundary discretization
    // density is the axis that still degrades: at 218 boundary points (angle
    // threshold pi/64) the residual is 30 holes, which is OPE-187 and open.

    Meshing::SurfaceMesh3DQualitySettings quality;

    Meshing::SurfaceMesher3D mesher(converter.getGeometryCollection(),
                                    converter.getTopology(),
                                    discretizationSettings,
                                    quality,
                                    Meshing::SurfaceMeshingStrategy::AmbientRCDT);
    auto surfaceMesh = mesher.mesh();

    std::cout << "SurfaceMesh3D: " << surfaceMesh.nodes.size() << " nodes, "
              << surfaceMesh.triangles.size() << " triangles\n";

    exporter.writeSurfaceMesh(surfaceMesh, "SaddleSurfaceMesh.vtu");
    Export::TsvExporter::writeSurfaceMesh(surfaceMesh, "SaddleSurfaceMesh");
    std::cout << "Exported refined mesh to SaddleSurfaceMesh.vtu\n";

    // Edge multiplicity across the restricted face set. An edge shared by
    // exactly 2 triangles is a normal closed-manifold interior edge -- that
    // includes the creases, where the two triangles come from different CAD
    // surfaces.
    //
    // This sees holes and excess faces only. It is blind to the third defect
    // kind, an edge carrying the expected NUMBER of faces but on the wrong
    // surfaces, so it under-reports -- RCDTMesher::runPipeline's own log line,
    // which classifies all three, is the authority for any number quoted from
    // this model.
    std::map<std::pair<size_t, size_t>, int> edgeMultiplicity;
    for (const auto& triangle : surfaceMesh.triangles)
        for (size_t i = 0; i < 3; ++i)
        {
            size_t a = triangle[i];
            size_t b = triangle[(i + 1) % 3];
            if (a > b)
                std::swap(a, b);
            edgeMultiplicity[{a, b}]++;
        }

    int holeEdgeCount = 0;
    int excessEdgeCount = 0;
    for (const auto& [edge, faceCount] : edgeMultiplicity)
    {
        if (faceCount < 2)
            ++holeEdgeCount;
        else if (faceCount > 2)
            ++excessEdgeCount;
    }
    std::cout << "Non-manifold edges: " << holeEdgeCount << " holes, " << excessEdgeCount
              << " excess (surface mismatches not visible here -- see the runPipeline log)\n";

    return 0;
}
