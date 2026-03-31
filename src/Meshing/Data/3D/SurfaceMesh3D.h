#pragma once

#include "Common/Types.h"
#include <array>
#include <map>
#include <string>
#include <vector>

namespace Meshing
{

/**
 * @brief Result of the 3D surface mesher
 *
 * A conforming triangle mesh of all CAD boundary surfaces, produced by
 * meshing each CAD face independently in UV space and enforcing conformity
 * on shared edges.
 *
 * Node IDs are indices into the nodes vector.
 * Triangle IDs are indices into the triangles vector.
 */
struct SurfaceMesh3D
{
    /// Global node coordinates. Index in vector == node ID.
    std::vector<Point3D> nodes;

    /// Surface triangles as node ID triplets. Index in vector == triangle ID.
    std::vector<std::array<size_t, 3>> triangles;

    /// Per-face groups: CAD surface ID → triangle IDs belonging to that face.
    std::map<std::string, std::vector<size_t>> faceTriangleIds;

    /// Edge node ordering: CAD edge ID → ordered node IDs along that edge
    /// (includes both endpoints, follows the edge's natural orientation).
    std::map<std::string, std::vector<size_t>> edgeNodeIds;
};

} // namespace Meshing
