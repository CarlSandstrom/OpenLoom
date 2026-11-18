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
 * A conforming triangle mesh of all CAD boundary surfaces: the restricted
 * facets of the ambient RCDT mesher (RCDTMesher).
 *
 * Node IDs are indices into the nodes vector.
 * Triangle IDs are indices into the triangles vector.
 */
struct SurfaceMesh3D
{
    /// Global node coordinates. Index in vector == node ID. Every node is used
    /// by a triangle or an edge chain.
    std::vector<Point3D> nodes;

    /// Surface triangles as node ID triplets. Index in vector == triangle ID.
    std::vector<std::array<NodeId, 3>> triangles;

    /// Per-face groups: CAD surface ID → triangle IDs belonging to that face.
    std::map<std::string, std::vector<size_t>> faceTriangleIds;

    /// Edge node ordering: CAD edge ID → ordered node IDs along that edge
    /// (includes both endpoints, follows the edge's natural orientation).
    std::map<std::string, std::vector<NodeId>> edgeNodeIds;
};

} // namespace Meshing
