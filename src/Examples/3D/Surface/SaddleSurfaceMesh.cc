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
    Geometry3D::DiscretizationSettings3D discSettings(std::nullopt, std::numbers::pi / 8.0, 2);

    Meshing::BoundaryDiscretizer3D discretizer(converter.getGeometryCollection(),
                                               converter.getTopology(),
                                               discSettings);
    discretizer.discretize();
    auto discResult = discretizer.releaseDiscretizationResult();

    std::cout << "Points:         " << discResult->points.size() << "\n";
    std::cout << "Topology edges: " << discResult->edgeIdToPointIndicesMap.size() << "\n";

    Export::VtkExporter exporter;
    exporter.writeEdgeMesh(*discResult, "SaddleSurfaceMeshEdges.vtu");
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
    // Known limitation (OPE-176): the finished mesh still has non-manifold
    // "hole" edges, concentrated on the creases between the saddle top and
    // its four side/bottom neighbors. RestrictedTriangulation::classifyFace()
    // has to pick which of two candidate surfaces a crease-straddling face
    // belongs to, using a discrete tessellation of each surface as an exact
    // crossing oracle; right at a shared boundary that's an inherently
    // ambiguous, all-or-nothing call, no matter how fine the oracle's grid
    // is. Priority 4's repair loop narrows the failure window by densifying
    // nearby, but can't close it -- it hits the same minimum-edge-length
    // floor and gives up, leaving a genuine gap rather than an infinite
    // retry. Weighted Delaunay refinement with protecting balls around every
    // curve segment (see CurveProtectionScheme, RCDTMesher::buildInitial())
    // keeps creases as explicit, structurally protected features instead of
    // repairing straddling faces after the fact, and has cut this count from
    // 863 (pre-OPE-176) to 126. classifyFace() now also leans on that
    // guarantee directly: a face whose vertex classification narrows to
    // exactly one surface, on a protected edge (two nodes chain-adjacent
    // along the same curve -- see CurveSegmentManager), is trusted without
    // the crossing oracle's confirmation PROVIDED it's the unique such
    // candidate across the whole edge star -- verified locally by requiring
    // any rival to pass the oracle itself, since the oracle is only
    // unreliable in the near-degenerate case right at the true crease, not
    // several tets away (see RestrictedTriangulation::
    // isUniqueEdgeStarCandidate()). That closes the specific "sole candidate,
    // oracle false negative" failure mode and has cut this count further,
    // from 126 to 84. The remaining 84 haven't been root-caused -- not yet
    // closed.
    Meshing::SurfaceMesh3DQualitySettings quality;

    Meshing::SurfaceMesher3D mesher(converter.getGeometryCollection(),
                                    converter.getTopology(),
                                    discSettings,
                                    quality,
                                    Meshing::SurfaceMeshingStrategy::AmbientRCDT);
    auto surfaceMesh = mesher.mesh();

    std::cout << "SurfaceMesh3D: " << surfaceMesh.nodes.size() << " nodes, "
              << surfaceMesh.triangles.size() << " triangles\n";

    exporter.writeSurfaceMesh(surfaceMesh, "SaddleSurfaceMesh.vtu");
    std::cout << "Exported refined mesh to SaddleSurfaceMesh.vtu\n";

    // Non-manifold hole edge count -- the OPE-176 verification target this
    // stress test exists to check (see the "Known limitation" note above).
    // An edge shared by exactly 2 triangles is a normal closed-manifold
    // interior edge; any other count is a hole (< 2) or self-intersection
    // (> 2) in the restricted face set.
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
    int nonManifoldEdgeCount = 0;
    for (const auto& [edge, count] : edgeMultiplicity)
        if (count != 2)
            ++nonManifoldEdgeCount;
    std::cout << "Non-manifold hole edges: " << nonManifoldEdgeCount << "\n";

    return 0;
}
