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
 * Refinement is not expected to meet the quality bound everywhere near a
 * genuinely small (20 deg) input angle: facets touching a protecting ball are
 * exempt from the angle criterion, as in CGAL Mesh_3. The output mesh itself
 * is a closed surface with no degenerate or duplicate-coordinate triangles.
 *
 * Topology:
 *   - 8 faces: the two flanges' outer faces (the crease sides), their two
 *     inner faces, two flange end strips, and the two end caps (the
 *     extruded profile's start/end)
 *   - The crease itself is the sharp (20 deg) edge shared by the two outer
 *     faces at the convex apex; the inner faces meet at the concave apex
 *
 * Exports:
 *   - SharpCreaseBracketEdges.vtu   : discretized boundary edges (color by EdgeID)
 *   - SharpCreaseBracketRefined.vtu : final refined surface mesh (color by SurfaceID)
 */

#include "Common/Logging.h"
#include "Common/Types.h"
#include "Export/TsvExporter.h"
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
#include <cmath>
#include <iostream>
#include <numbers>
#include <optional>

namespace
{

constexpr double FLANGE_LENGTH = 15.0;
constexpr double FLANGE_THICKNESS = 2.0;
constexpr double DEPTH = 10.0;
constexpr double BEND_ANGLE_DEGREES = 20.0; // < 30 deg: the acute dihedral under test

using Meshing::Point2D;

// Left-normal of a direction (rotate 90 deg counter-clockwise).
Point2D leftNormal(const Point2D& direction)
{
    return {-direction.y(), direction.x()};
}

// Intersection of line (point1 + t*direction1) with line (point2 + s*direction2).
Point2D intersectLines(const Point2D& point1, const Point2D& direction1, const Point2D& point2, const Point2D& direction2)
{
    double denominator = direction1.x() * direction2.y() - direction1.y() * direction2.x();
    double t = ((point2.x() - point1.x()) * direction2.y() - (point2.y() - point1.y()) * direction2.x()) / denominator;
    return point1 + direction1 * t;
}

// Builds the bent-strip cross-section: two THICKNESS-wide flanges of length
// FLANGE_LENGTH, centerlines meeting at the origin with BEND_ANGLE_DEGREES
// between them, mitered cleanly at both the concave (inner) and convex
// (outer) corners.
std::array<Point2D, 6> buildCrossSection()
{
    double halfAngle = (BEND_ANGLE_DEGREES / 2.0) * std::numbers::pi / 180.0;
    double halfThickness = FLANGE_THICKNESS / 2.0;

    Point2D direction1{std::cos(-halfAngle), std::sin(-halfAngle)};
    Point2D direction2{std::cos(halfAngle), std::sin(halfAngle)};
    // Each normal points into the concave (inner) side, toward the other
    // flange: left of flange 1 (running at -halfAngle), right of flange 2
    // (running at +halfAngle). Using leftNormal for both put the two "inner"
    // edges on the same side and made the profile a self-intersecting bow-tie
    // (signed area 0), so the extruded solid passed through itself along
    // x = 5.76, y = 0 -- invalid for BRepCheck_Analyzer, its end caps
    // untriangulable by BRepMesh.
    Point2D normal1 = leftNormal(direction1);
    Point2D normal2 = leftNormal(direction2) * -1.0;

    // Concave miter: where the two flanges' inner (facing each other) edges cross.
    Point2D innerApex = intersectLines(normal1 * halfThickness, direction1, normal2 * halfThickness, direction2);
    // Convex miter: where the two flanges' outer edges cross (behind the origin).
    Point2D outerApex = intersectLines(normal1 * -halfThickness, direction1, normal2 * -halfThickness, direction2);

    return {
        outerApex,
        direction1 * FLANGE_LENGTH + normal1 * -halfThickness, // flange 1, outer, far end
        direction1 * FLANGE_LENGTH + normal1 * halfThickness,  // flange 1, inner, far end
        innerApex,
        direction2 * FLANGE_LENGTH + normal2 * halfThickness,  // flange 2, inner, far end
        direction2 * FLANGE_LENGTH + normal2 * -halfThickness, // flange 2, outer, far end
    };
}

TopoDS_Shape buildSharpCreaseBracket()
{
    auto profile = buildCrossSection();

    BRepBuilderAPI_MakePolygon polygonBuilder;
    for (auto& point : profile)
        polygonBuilder.Add(gp_Pnt(point.x(), point.y(), 0.0));
    polygonBuilder.Close();

    TopoDS_Face face = BRepBuilderAPI_MakeFace(polygonBuilder.Wire()).Face();
    return BRepPrimAPI_MakePrism(face, gp_Vec(0.0, 0.0, DEPTH)).Shape();
}

} // namespace

int main()
{
    Common::initializeLogging();

    TopoDS_Shape shape = buildSharpCreaseBracket();
    Readers::TopoDS_ShapeConverter converter(shape);

    Geometry3D::DiscretizationSettings3D discretizationSettings(std::nullopt, std::numbers::pi / 8.0, 2);

    // Export the raw boundary discretization for inspection near the crease,
    // independent of the meshing strategy used below.
    auto discretizationResult =
        Meshing::BoundaryDiscretizer3D::discretize(converter.getGeometryCollection(),
                                                   converter.getTopology(),
                                                   discretizationSettings);

    std::cout << "Points:         " << discretizationResult->points.size() << "\n";
    std::cout << "Topology edges: " << discretizationResult->edgeIdToPointIndicesMap.size() << "\n";

    Export::VtkExporter exporter;
    exporter.writeEdgeMesh(*discretizationResult, "SharpCreaseBracketEdges.vtu");
    Export::TsvExporter::writeDiscretization(*discretizationResult, "SharpCreaseBracketEdges");
    std::cout << "Exported edge mesh to SharpCreaseBracketEdges.vtu (color by EdgeID)\n";

    Meshing::SurfaceMesher3D mesher(converter.getGeometryCollection(),
                                    converter.getTopology(),
                                    discretizationSettings,
                                    Meshing::SurfaceMesh3DQualitySettings{});

    mesher.mesh();
    auto& surfaceMesh = *mesher.getMesh();

    std::cout << "SurfaceMesh3D: " << surfaceMesh.nodes.size() << " nodes, "
              << surfaceMesh.triangles.size() << " triangles\n";

    exporter.writeSurfaceMesh(surfaceMesh, "SharpCreaseBracketRefined.vtu");
    Export::TsvExporter::writeSurfaceMesh(surfaceMesh, "SharpCreaseBracketRefined");
    std::cout << "Exported refined mesh to SharpCreaseBracketRefined.vtu (color by SurfaceID)\n";

    return 0;
}
