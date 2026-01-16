#pragma once

#include "Common/Types.h"
#include "Meshing/Core/3D/GeometryStructures3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"
#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace Geometry3D
{
class IEdge3D;
class ISurface3D;
} // namespace Geometry3D

namespace Meshing
{

class MeshMutator3D;
class MeshConnectivity;

/**
 * @brief General mesh operations for 3D meshes
 *
 * Provides high-level mesh operations that build upon MeshMutator3D.
 * Handles algorithms like Bowyer-Watson 3D insertion, cavity finding,
 * and constraint operations for Shewchuk's refinement algorithm.
 */
class MeshOperations3D
{
public:
    /**
     * @brief Construct mesh operations with mesh data
     * @param meshData Reference to the 3D mesh data
     */
    explicit MeshOperations3D(MeshData3D& meshData);

    /**
     * @brief Create a bounding tetrahedron that contains all given points
     *
     * Creates a large tetrahedron that encloses all input points with margin.
     * This is the first step in Delaunay initialization.
     *
     * @param points The points that must be contained within the tetrahedron
     * @return Array of 4 node IDs for the bounding tetrahedron vertices
     */
    std::array<size_t, 4> createBoundingTetrahedron(const std::vector<Point3D>& points);

    /**
     * @brief Initialize Delaunay triangulation with a set of points
     *
     * Creates a bounding tetrahedron, inserts all vertices using Bowyer-Watson,
     * and stores the bounding vertex IDs for later removal.
     *
     * @param points The points to triangulate
     * @return Vector of node IDs for the inserted points (in same order as input)
     */
    std::vector<size_t> initializeDelaunay(const std::vector<Point3D>& points);

    /**
     * @brief Remove the bounding tetrahedron vertices and connected elements
     *
     * Removes all tetrahedra that contain any of the bounding vertices,
     * then removes the bounding vertices themselves.
     *
     * @param boundingNodeIds The 4 node IDs of the bounding tetrahedron
     */
    void removeBoundingTetrahedron(const std::array<size_t, 4>& boundingNodeIds);

    /**
     * @brief Insert a vertex using 3D Bowyer-Watson algorithm
     *
     * Finds conflicting tetrahedra, removes them to form a cavity,
     * and retriangulates with the new vertex. Maintains Delaunay property.
     *
     * @param point The 3D point to insert
     * @param edgeParameters Optional parametric coordinates on parent edges
     * @param edgeIds Optional geometry entity IDs this vertex belongs to
     * @return Node ID of the inserted vertex
     */
    size_t insertVertexBowyerWatson(const Point3D& point,
                                    const std::vector<double>& edgeParameters = {},
                                    const std::vector<std::string>& edgeIds = {});

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
    std::vector<std::array<size_t, 3>> findCavityBoundary(
        const std::vector<size_t>& conflictingIndices) const;

    /**
     * @brief Split a constrained subsegment at its parametric midpoint
     *
     * Uses the parent edge geometry to find the correct midpoint on curved edges.
     * Inserts the new node via Bowyer-Watson and updates constraint lists.
     *
     * @param subsegment The constrained subsegment to split
     * @param parentEdge The geometric edge the subsegment lies on
     * @return Pair of new subsegments, or nullopt if split failed
     */
    std::optional<std::pair<ConstrainedSubsegment3D, ConstrainedSubsegment3D>>
    splitConstrainedSubsegment(const ConstrainedSubsegment3D& subsegment,
                               const Geometry3D::IEdge3D& parentEdge);

    /**
     * @brief Split a constrained subfacet at its circumcenter
     *
     * Computes the circumcenter of the triangle and inserts it as a new vertex.
     * The subfacet will be subdivided into smaller subfacets.
     *
     * @param subfacet The constrained subfacet to split
     * @param parentSurface The geometric surface the subfacet lies on
     * @return ID of the inserted vertex, or nullopt if split failed
     */
    std::optional<size_t> splitConstrainedSubfacet(
        const ConstrainedSubfacet3D& subfacet,
        const Geometry3D::ISurface3D& parentSurface);

    /**
     * @brief Remove tetrahedra that contain a specific node
     *
     * Used when removing a node or preparing for vertex re-insertion.
     *
     * @param nodeId The node ID
     * @return true if any tetrahedra were removed
     */
    bool removeTetrahedraContainingNode(size_t nodeId);

    /**
     * @brief Classify tetrahedra as interior/exterior using flood fill
     *
     * Uses mesh topology (constraint faces) to determine which tetrahedra
     * are inside the domain vs outside or in holes. Removes exterior tets.
     *
     * @param constrainedSubfacets Vector of constraint faces forming domain boundaries
     */
    void classifyTetrahedraInteriorExterior(
        const std::vector<ConstrainedSubfacet3D>& constrainedSubfacets);

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
     * @brief Get the mesh mutator for primitive operations
     */
    MeshMutator3D& getMutator() { return *mutator_; }
    const MeshMutator3D& getMutator() const { return *mutator_; }

private:
    MeshData3D& meshData_;
    std::unique_ptr<MeshMutator3D> mutator_;

    /**
     * @brief Retriangulate the cavity with the new vertex
     *
     * Creates new tetrahedra connecting the vertex to the cavity boundary.
     *
     * @param vertexNodeId The newly inserted vertex
     * @param boundary The triangular faces forming the cavity boundary
     */
    void retriangulate(size_t vertexNodeId,
                       const std::vector<std::array<size_t, 3>>& boundary);

    /**
     * @brief Create ordered edge key for lookups
     */
    static std::pair<size_t, size_t> makeEdgeKey(size_t a, size_t b)
    {
        return (a < b) ? std::make_pair(a, b) : std::make_pair(b, a);
    }

    /**
     * @brief Create ordered triangle key for lookups
     */
    static std::array<size_t, 3> makeTriangleKey(size_t a, size_t b, size_t c);
};

} // namespace Meshing
