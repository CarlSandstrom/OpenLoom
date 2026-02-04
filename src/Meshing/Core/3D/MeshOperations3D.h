#pragma once

#include "Common/Types.h"
#include "Meshing/Core/3D/GeometryStructures3D.h"
#include "Meshing/Core/3D/MeshQueries3D.h"
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
 * @brief Mutation operations for 3D meshes
 *
 * Provides high-level mesh operations that modify the mesh, building upon MeshMutator3D.
 * Handles algorithms like Bowyer-Watson 3D insertion, cavity finding,
 * and constraint operations for Shewchuk's refinement algorithm.
 * Uses MeshQueries3D for read-only query operations.
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
     * @brief Split a constrained subsegment at its parametric midpoint
     *
     * Uses the parent edge geometry to find the correct midpoint on curved edges.
     * Inserts the new node via Bowyer-Watson and updates constraint lists.
     *
     * @param subsegment The constrained subsegment to split
     * @param parentEdge The geometric edge the subsegment lies on
     * @return Pair of new subsegments, or nullopt if split failed
     */
    std::optional<std::pair<ConstrainedSubsegment3D, ConstrainedSubsegment3D>> splitConstrainedSubsegment(
        const ConstrainedSubsegment3D& subsegment,
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
    std::optional<size_t> splitConstrainedSubfacet(const ConstrainedSubfacet3D& subfacet,
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
     * Uses constraint faces from MeshData3D to determine which tetrahedra
     * are inside the domain vs outside or in holes. Removes exterior tets.
     */
    void classifyTetrahedraInteriorExterior();

    /**
     * @brief Get the mesh mutator for primitive operations
     */
    MeshMutator3D& getMutator() { return *mutator_; }
    const MeshMutator3D& getMutator() const { return *mutator_; }

    /**
     * @brief Get the mesh queries for read-only operations
     */
    MeshQueries3D& getQueries() { return queries_; }
    const MeshQueries3D& getQueries() const { return queries_; }

private:
    MeshData3D& meshData_;
    MeshQueries3D queries_;
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
};

} // namespace Meshing
