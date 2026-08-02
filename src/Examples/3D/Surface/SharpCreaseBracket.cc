/**
 * @file SharpCreaseBracket.cc
 * @brief Stress test: surface mesh of a bent-plate solid with a sharp
 * (acute, <30 deg) dihedral crease between two planar flanges.
 *
 * Built as a single extruded polygon (bent-strip cross-section, mitered at
 * both the convex and concave corners) rather than via a boolean fuse of two
 * rotated boxes: rotating a box about an axis that lies exactly on one of its
 * own edges makes the two solids overlap tangentially along most of their
 * length, which OCC's boolean fuse turns into fragmented, coincident
 * topology at the shared edge (confirmed empirically: that construction
 * produced 161 zero-area triangles, with dozens of discretization points
 * collapsing onto the same coordinate). Extruding a single well-defined
 * polygon has no such degeneracy.
 *
 * The cross-section is two THICKNESS-wide flanges of length FLANGE_LENGTH,
 * meeting at the origin with an interior angle of BEND_ANGLE_DEGREES between
 * their centerlines. Extruding it DEPTH along Z gives a solid whose two large
 * flat outer faces meet along the extrusion direction at a dihedral angle of
 * BEND_ANGLE_DEGREES -- an acute feature line RCDT must discretize and refine
 * without losing conformity across the crease.
 *
 * Retargeted from the legacy per-face UV-space pipeline to SurfaceMesher3D's
 * AmbientRCDT strategy (OPE-120), since the UV-space path is being removed
 * (OPE-166) and crease handling is mesher-agnostic.
 *
 * This example is what surfaced a real RCDTRefiner bug (fixed alongside it):
 * priority-1 (encroached curve segment splitting) had no minimum-size floor,
 * unlike priorities 2 and 3. Near the acute pocket between the two flanges, a
 * segment could get bisected forever -- each split's midpoint still fell
 * inside the same nearby vertex's encroachment sphere, so the "new" segment
 * was encroached again next iteration -- producing hundreds of duplicate
 * nodes at one coordinate. See RCDTRefiner::unrefinableSegments_.
 *
 * Refinement still hits RCDTRefiner's 500-iteration safety cap here rather
 * than fully converging to the quality bound -- expected for a genuinely
 * small (20 deg) input angle, the same documented class of limitation as the
 * sliver tetrahedra RCDTRefiner's tet-quality phase doesn't fully solve
 * either (see RCDTRefiner's class doc comment). The output mesh itself is
 * valid throughout (no degenerate or duplicate-coordinate triangles).
 *
 * Topology:
 *   - 6 faces: two DEPTH-long flange faces (the crease sides), two end caps
 *     (the extruded profile's start/end), two flange end-cap strips
 *   - The crease itself is the sharp edge shared by the two flange faces
 *
 * Exports:
 *   - SharpCreaseBracketEdges.vtu   : discretized boundary edges (color by EdgeID)
 *   - SharpCreaseBracketRefined.vtu : final refined surface mesh (color by SurfaceID)
 */

#include "Common/Logging.h"
#include "Export/VtkExporter.h"
#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/General/BoundaryDiscretizer3D.h"
#include "Meshing/Core/3D/Surface/SurfaceMesher3D.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"
#include "Readers/OpenCascade/TopoDS_ShapeConverter.h"

#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <array>
#include <iostream>
#include <numbers>

namespace
{

constexpr double FLANGE_LENGTH = 15.0;
constexpr double FLANGE_THICKNESS = 2.0;
constexpr double DEPTH = 10.0;
constexpr double BEND_ANGLE_DEGREES = 20.0; // < 30 deg: the acute dihedral under test

using Point2D = std::array<double, 2>;

Point2D operator+(const Point2D& a, const Point2D& b) { return {a[0] + b[0], a[1] + b[1]}; }
Point2D operator*(const Point2D& v, double s) { return {v[0] * s, v[1] * s}; }

// Left-normal of a direction vector (rotate 90 deg counter-clockwise).
Point2D leftNormal(const Point2D& d) { return {-d[1], d[0]}; }

// Intersection of line (p1 + t*d1) with line (p2 + s*d2).
Point2D intersectLines(const Point2D& p1, const Point2D& d1, const Point2D& p2, const Point2D& d2)
{
    const double denom = d1[0] * d2[1] - d1[1] * d2[0];
    const double t = ((p2[0] - p1[0]) * d2[1] - (p2[1] - p1[1]) * d2[0]) / denom;
    return p1 + d1 * t;
}

// Builds the bent-strip cross-section: two THICKNESS-wide flanges of length
// FLANGE_LENGTH, centerlines meeting at the origin with BEND_ANGLE_DEGREES
// between them, mitered cleanly at both the concave (inner) and convex
// (outer) corners.
std::array<Point2D, 6> buildCrossSection()
{
    const double halfAngle = (BEND_ANGLE_DEGREES / 2.0) * std::numbers::pi / 180.0;
    const double halfThickness = FLANGE_THICKNESS / 2.0;

    const Point2D dir1{std::cos(-halfAngle), std::sin(-halfAngle)};
    const Point2D dir2{std::cos(halfAngle), std::sin(halfAngle)};
    const Point2D n1 = leftNormal(dir1); // points into the concave (inner) side
    const Point2D n2 = leftNormal(dir2);

    // Concave miter: where the two flanges' inner (facing each other) edges cross.
    const Point2D innerApex = intersectLines(n1 * halfThickness, dir1, n2 * halfThickness, dir2);
    // Convex miter: where the two flanges' outer edges cross (behind the origin).
    const Point2D outerApex = intersectLines(n1 * -halfThickness, dir1, n2 * -halfThickness, dir2);

    return {
        outerApex,
        dir1 * FLANGE_LENGTH + n1 * -halfThickness, // flange 1, outer, far end
        dir1 * FLANGE_LENGTH + n1 * halfThickness,  // flange 1, inner, far end
        innerApex,
        dir2 * FLANGE_LENGTH + n2 * halfThickness,  // flange 2, inner, far end
        dir2 * FLANGE_LENGTH + n2 * -halfThickness, // flange 2, outer, far end
    };
}

TopoDS_Shape buildSharpCreaseBracket()
{
    const auto profile = buildCrossSection();

    BRepBuilderAPI_MakePolygon polygonBuilder;
    for (const auto& point : profile)
        polygonBuilder.Add(gp_Pnt(point[0], point[1], 0.0));
    polygonBuilder.Close();

    TopoDS_Face face = BRepBuilderAPI_MakeFace(polygonBuilder.Wire()).Face();
    return BRepPrimAPI_MakePrism(face, gp_Vec(0.0, 0.0, DEPTH)).Shape();
}

} // namespace

int main()
{
    Common::initLogging();

    TopoDS_Shape shape = buildSharpCreaseBracket();
    Readers::TopoDS_ShapeConverter converter(shape);

    Geometry3D::DiscretizationSettings3D discSettings(std::nullopt, std::numbers::pi / 8.0, 2);

    // Export the raw boundary discretization for inspection near the crease,
    // independent of the meshing strategy used below.
    Meshing::BoundaryDiscretizer3D discretizer(converter.getGeometryCollection(),
                                               converter.getTopology(),
                                               discSettings);
    discretizer.discretize();
    auto discResult = discretizer.releaseDiscretizationResult();

    std::cout << "Points:         " << discResult->points.size() << "\n";
    std::cout << "Topology edges: " << discResult->edgeIdToPointIndicesMap.size() << "\n";

    Export::VtkExporter exporter;
    exporter.writeEdgeMesh(*discResult, "SharpCreaseBracketEdges.vtu");
    std::cout << "Exported edge mesh to SharpCreaseBracketEdges.vtu (color by EdgeID)\n";

    // Force AmbientRCDT: this shape has no periodic/seam surfaces, so Auto
    // would otherwise resolve to the legacy per-face UV-space pipeline.
    Meshing::SurfaceMesher3D mesher(converter.getGeometryCollection(),
                                    converter.getTopology(),
                                    discSettings,
                                    Meshing::SurfaceMesh3DQualitySettings{},
                                    Meshing::SurfaceMeshingStrategy::AmbientRCDT);

    auto surfaceMesh = mesher.mesh();

    std::cout << "SurfaceMesh3D: " << surfaceMesh.nodes.size() << " nodes, "
              << surfaceMesh.triangles.size() << " triangles\n";

    exporter.writeSurfaceMesh(surfaceMesh, "SharpCreaseBracketRefined.vtu");
    std::cout << "Exported refined mesh to SharpCreaseBracketRefined.vtu (color by SurfaceID)\n";

    return 0;
}
