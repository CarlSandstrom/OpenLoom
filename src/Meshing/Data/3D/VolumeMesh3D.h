#pragma once

#include "Common/Types.h"
#include <array>
#include <map>
#include <string>
#include <vector>

namespace Meshing
{

/**
 * @brief Result of a 3D volume mesher
 *
 * A conforming tetrahedral mesh of a CAD solid's interior, together with its
 * labeled boundary (the surface a volume mesher's boundary faces belong to —
 * needed downstream for e.g. FEM boundary condition assignment).
 *
 * Node IDs are indices into the nodes vector. Boundary triangle IDs are
 * indices into the boundaryTriangles vector. Nodes are shared between
 * tetrahedra and boundaryTriangles — there is exactly one nodes vector.
 */
struct VolumeMesh3D
{
    /// Global node coordinates. Index in vector == node ID.
    std::vector<Point3D> nodes;

    /// Tetrahedra as node ID quadruplets. Index in vector == tetrahedron ID.
    std::vector<std::array<size_t, 4>> tetrahedra;

    /// Boundary surface triangles as node ID triplets (the tetrahedra's outer
    /// faces that lie on a CAD surface). Index in vector == triangle ID.
    std::vector<std::array<size_t, 3>> boundaryTriangles;

    /// Per-face groups: CAD surface ID → boundary triangle IDs belonging to that face.
    std::map<std::string, std::vector<size_t>> boundaryFaceTriangleIds;

    /// Edge node ordering: CAD edge ID → ordered node IDs along that edge
    /// (includes both endpoints, follows the edge's natural orientation).
    std::map<std::string, std::vector<size_t>> boundaryEdgeNodeIds;
};

} // namespace Meshing
