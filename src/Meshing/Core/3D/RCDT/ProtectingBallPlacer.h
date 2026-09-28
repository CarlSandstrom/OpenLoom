#pragma once

#include <cstddef>
#include <unordered_map>

namespace Geometry3D
{
class GeometryCollection3D;
} // namespace Geometry3D

namespace Topology3D
{
class Topology3D;
} // namespace Topology3D

namespace Meshing
{
struct DiscretizationResult3D;
} // namespace Meshing

namespace Meshing
{

/**
 * @brief Places the protected points and ball radii on every CAD corner and
 * curve the way CGAL Mesh_3 does (Protect_edges_sizing_field): the protection
 * used by the CGAL-style refinement path (OPE-186).
 *
 *  - Corners: radius from the size function, capped at a third of the
 *    distance to the nearest other corner so corner balls stay disjoint.
 *  - Curves (insert_balls): between two balls of radii sp <= sq a curve
 *    distance d apart, n = round(2(d - sq) / (sp + sq)) balls whose radii grow
 *    linearly from sp to sq, each spaced from the last by its own radius. A
 *    long run is split at its midpoint with the local size first, so the size
 *    function is followed rather than interpolated between the curve's ends.
 *  - Repair (refine_balls): two balls that intersect without being
 *    neighbours on a curve are both shrunk to at most their distance / 2.1,
 *    and neighbours that no longer overlap along the curve are repopulated
 *    between; repeated until nothing changes, at most 29 rounds.
 *
 * The size function is the spacing of the existing boundary discretization
 * along each curve.
 *
 * Rewrites discretization in place, replacing every protected curve's
 * interior points (corners and surface points are kept), and returns the
 * weight (squared radius) of every protected point by index. Seam twins get
 * their original edge's points in reverse; degenerate edges are left as they
 * are.
 */
class ProtectingBallPlacer
{
public:
    static std::unordered_map<std::size_t, double> place(DiscretizationResult3D& discretization,
                                                         const Topology3D::Topology3D& topology,
                                                         const Geometry3D::GeometryCollection3D& geometry);
};

} // namespace Meshing
