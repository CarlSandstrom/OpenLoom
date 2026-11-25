#pragma once

#include "Common/Types.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/TetrahedronElement.h"
#include <array>
#include <optional>
#include <vector>

namespace Meshing
{

/**
 * @brief Query operations for 3D meshes (read-only)
 *
 * Provides methods for querying mesh data without modifying it.
 * Handles algorithms like finding conflicting tetrahedra, cavity boundaries,
 * and face adjacency lookups.
 */
class MeshQueries3D
{
public:
    /**
     * @brief Construct mesh queries with mesh data
     * @param meshData Reference to the 3D mesh data (read-only)
     */
    explicit MeshQueries3D(const MeshData3D& meshData);

    /**
     * @brief Find tetrahedra whose orthogonal sphere contains the (possibly
     * weighted) point, connected through shared faces to one that contains
     * the point -- the point's conflict region. Empty for a point outside
     * every tetrahedron. Exact ties count as conflicts, so a tie far from
     * the point could otherwise join the cavity as a separate piece
     * (OPE-251).
     *
     * Uses RegularPredicates3D's weighted in-sphere ("orthosphere") test,
     * incorporating each candidate tetrahedron's own vertices' weights (see
     * Node3D::getWeight()) as well as pointWeight. Reduces exactly to a
     * plain Delaunay circumsphere test when every weight involved is 0 --
     * the default for pointWeight and for every node until OPE-176's
     * crease-protection scheme starts assigning nonzero weights.
     *
     * @param point The point to test
     * @param pointWeight The point's own weight (0 for an unweighted query)
     * @return IDs of conflicting tetrahedra
     */
    std::vector<ElementId> findConflictingTetrahedra(const Point3D& point, double pointWeight = 0.0) const;

    /**
     * @brief Find the boundary of the cavity formed by conflicting tetrahedra
     * @param conflictingIndices Indices of conflicting tetrahedra
     * @return Boundary triangular faces of the cavity (as node ID triplets)
     */
    std::vector<std::array<NodeId, 3>> findCavityBoundary(const std::vector<ElementId>& conflictingIndices) const;

    /**
     * @brief Find all tetrahedra containing a specific face
     *
     * Returns the IDs of all tetrahedra that have all three specified nodes
     * as vertices (at most 2 tetrahedra share a face).
     *
     * @param nodeId1 First node ID of the face
     * @param nodeId2 Second node ID of the face
     * @param nodeId3 Third node ID of the face
     * @return Vector of tetrahedron IDs containing the face
     */
    std::vector<ElementId> findTetrahedraWithFace(NodeId nodeId1, NodeId nodeId2, NodeId nodeId3) const;

    /**
     * @brief Find the opposite vertex of a face in a tetrahedron
     *
     * Given a tetrahedron and three nodes forming a face, returns the
     * fourth node (the apex opposite to that face).
     *
     * @param tetId The tetrahedron ID
     * @param faceNode1 First node of the face
     * @param faceNode2 Second node of the face
     * @param faceNode3 Third node of the face
     * @return Node ID of the opposite vertex, or std::nullopt if not found
     */
    std::optional<NodeId> findOppositeVertex(ElementId tetId, NodeId faceNode1, NodeId faceNode2, NodeId faceNode3) const;

private:
    const MeshData3D& meshData_;
};

} // namespace Meshing
