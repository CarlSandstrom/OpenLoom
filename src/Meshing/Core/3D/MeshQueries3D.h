#pragma once

#include "Common/Types.h"
#include "Meshing/Core/3D/GeometryStructures3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"
#include <array>
#include <map>
#include <string>
#include <vector>

namespace Topology3D
{
class Topology3D;
}

namespace Meshing
{

/**
 * @brief Query operations for 3D meshes (read-only)
 *
 * Provides methods for querying mesh data without modifying it.
 * Handles algorithms like finding conflicting tetrahedra, cavity boundaries,
 * edge/face existence checks, and encroachment detection.
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
     * @brief Find tetrahedra whose circumsphere contains the point
     * @param point The point to test
     * @return IDs of conflicting tetrahedra
     */
    std::vector<size_t> findConflictingTetrahedra(const Point3D& point) const;

    /**
     * @brief Find the boundary of the cavity formed by conflicting tetrahedra
     * @param conflictingIndices Indices of conflicting tetrahedra
     * @return Boundary triangular faces of the cavity (as node ID triplets)
     */
    std::vector<std::array<size_t, 3>> findCavityBoundary(const std::vector<size_t>& conflictingIndices) const;

    /**
     * @brief Check if an edge exists in the mesh
     *
     * An edge exists if there is at least one tetrahedron that has both
     * node IDs as vertices.
     *
     * @param nodeId1 First node ID
     * @param nodeId2 Second node ID
     * @return true if the edge exists in the mesh
     */
    bool edgeExistsInMesh(size_t nodeId1, size_t nodeId2) const;

    /**
     * @brief Find all tetrahedra containing a specific edge
     *
     * Returns the IDs of all tetrahedra that have both specified nodes
     * as vertices.
     *
     * @param nodeId1 First node ID of the edge
     * @param nodeId2 Second node ID of the edge
     * @return Vector of tetrahedron IDs containing the edge
     */
    std::vector<size_t> findTetrahedraWithEdge(size_t nodeId1, size_t nodeId2) const;

    /**
     * @brief Check if a triangular face exists in the mesh
     *
     * A face exists if there is at least one tetrahedron that has all
     * three node IDs as vertices.
     *
     * @param nodeId1 First node ID
     * @param nodeId2 Second node ID
     * @param nodeId3 Third node ID
     * @return true if the face exists in the mesh
     */
    bool faceExistsInMesh(size_t nodeId1, size_t nodeId2, size_t nodeId3) const;

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
    std::vector<size_t> findTetrahedraWithFace(size_t nodeId1, size_t nodeId2, size_t nodeId3) const;

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
     * @return Node ID of the opposite vertex, or SIZE_MAX if not found
     */
    size_t findOppositeVertex(size_t tetId, size_t faceNode1, size_t faceNode2, size_t faceNode3) const;

    /**
     * @brief Find all skinny tetrahedra exceeding the quality bound
     *
     * Returns IDs of tetrahedra whose circumradius-to-shortest-edge ratio
     * exceeds the given bound. These are candidates for refinement.
     *
     * @param ratioBound The B ratio threshold (typically > 2)
     * @return Vector of tetrahedron IDs that are "skinny"
     */
    std::vector<size_t> findSkinnyTetrahedra(double ratioBound) const;

    /**
     * @brief Find encroached subsegments for a given point
     *
     * Checks all provided subsegments and returns those that would be
     * encroached if the given point were inserted.
     *
     * @param point The point to test
     * @param subsegments The subsegments to check
     * @return Vector of subsegments encroached by the point
     */
    std::vector<ConstrainedSubsegment3D> findEncroachingSubsegments(
        const Point3D& point,
        const std::vector<ConstrainedSubsegment3D>& subsegments) const;

    /**
     * @brief Find encroached subfacets for a given point
     *
     * Checks all provided subfacets and returns those that would be
     * encroached if the given point were inserted.
     *
     * @param point The point to test
     * @param subfacets The subfacets to check
     * @return Vector of subfacets encroached by the point
     */
    std::vector<ConstrainedSubfacet3D> findEncroachingSubfacets(
        const Point3D& point,
        const std::vector<ConstrainedSubfacet3D>& subfacets) const;

    /**
     * @brief Extract constrained subsegments from topology edges
     *
     * Creates a list of subsegments from the topology edges, mapping
     * discretization point indices to mesh node IDs. Each subsegment
     * represents a portion of a CAD edge that should eventually appear
     * as an edge in the tetrahedralization.
     *
     * @param topology The 3D topology containing edge definitions
     * @param cornerIdToPointIndexMap Maps corner IDs to discretization point indices
     * @param pointIndexToNodeIdMap Maps discretization point indices to mesh node IDs
     * @param edgeIdToPointIndicesMap Maps edge IDs to ordered discretization point indices
     * @return Vector of ConstrainedSubsegment3D representing all edge subsegments
     */
    std::vector<ConstrainedSubsegment3D> extractConstrainedSubsegments(
        const Topology3D::Topology3D& topology,
        const std::map<std::string, size_t>& cornerIdToPointIndexMap,
        const std::map<size_t, size_t>& pointIndexToNodeIdMap,
        const std::map<std::string, std::vector<size_t>>& edgeIdToPointIndicesMap) const;

private:
    const MeshData3D& meshData_;
};

} // namespace Meshing
