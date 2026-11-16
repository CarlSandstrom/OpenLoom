#pragma once

#include "Common/Types.h"

#include <array>
#include <cstddef>
#include <vector>

namespace Meshing
{
class MeshConnectivity;
class MeshData3D;
} // namespace Meshing

namespace Meshing
{

/**
 * @brief The conflict region of a point in the weighted (regular)
 * tetrahedralization, and CGAL's test for whether the point may be inserted
 * at all. Shared by the facet and tetrahedron levels of the CGAL-style
 * refinement (SurfaceDelaunayRefiner, TetrahedronDelaunayRefiner).
 */
namespace RegularConflictRegion
{

/// The tetrahedra in conflict with point, grown from seeds across shared
/// faces. In a regular triangulation that region is connected, so this finds
/// the same set as scanning every tetrahedron, at the cost of the region
/// alone. Empty when no seed conflicts. A seed of SIZE_MAX is ignored.
std::vector<std::size_t> find(const std::array<std::size_t, 2>& seeds,
                              const Point3D& point,
                              const MeshData3D& meshData,
                              const MeshConnectivity& connectivity);

/// Whether point would be hidden by, or coincide with, a vertex of the
/// tetrahedra it conflicts with: power distance |point - v|^2 - w_v <= 0. A
/// hidden point is not a vertex of the regular triangulation, so inserting it
/// breaks the triangulation; a coincident one duplicates a node. CGAL's
/// insert refuses both.
bool isHiddenOrDuplicate(const Point3D& point, const std::vector<std::size_t>& region, const MeshData3D& meshData);

} // namespace RegularConflictRegion

} // namespace Meshing
