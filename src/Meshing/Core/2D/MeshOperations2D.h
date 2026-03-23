#pragma once

#include "Common/Types.h"
#include "Meshing/Core/2D/ElementGeometry2D.h"
#include "Meshing/Core/2D/GeometryStructures2D.h"
#include "Meshing/Core/2D/MeshQueries2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include <array>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

namespace Geometry2D
{
class IEdge2D;
}

namespace Topology2D
{
class Topology2D;
}

namespace Meshing
{

class MeshMutator2D;
class PeriodicMeshData2D;

/**
 * @brief Mutation operations for 2D meshes
 *
 * Provides high-level mesh operations that modify the mesh, building upon MeshMutator2D.
 * Handles algorithms like Bowyer-Watson insertion, edge enforcement, and triangle classification.
 * Uses MeshQueries2D for read-only query operations.
 */
class MeshOperations2D
{
public:
    /**
     * @brief Construct mesh operations with mesh data
     * @param meshData Reference to the 2D mesh data
     */
    explicit MeshOperations2D(MeshData2D& meshData);

    /// Periodic-aware constructor.
    /// periodicData may be null (equivalent to the non-periodic constructor).
    MeshOperations2D(MeshData2D& meshData, PeriodicMeshData2D* periodicData);

    /**
     * @brief Insert a vertex using 2D Bowyer-Watson algorithm
     *
     * Finds conflicting triangles, removes them to form a cavity,
     * and retriangulates with the new vertex. Maintains Delaunay property.
     *
     * @param point The 2D point to insert
     * @param edgeParameters Optional parametric coordinates on parent edges
     * @param edgeIds Optional edge IDs this vertex belongs to
     * @return Node ID of the inserted vertex
     */
    size_t insertVertexBowyerWatson(const Point2D& point,
                                    const std::vector<double>& edgeParameters = {},
                                    const std::vector<std::string>& edgeIds = {});

    /**
     * @brief Remove all triangles containing a specific node
     * @param nodeId The node ID to search for
     * @return True if any triangles were removed
     */
    bool removeTrianglesContainingNode(size_t nodeId);

    /**
     * @brief Enforce an edge in the mesh
     *
     * Ensures the edge exists in the triangulation by removing intersecting
     * triangles and retriangulating the cavity.
     *
     * @param nodeId1 First node ID of the edge
     * @param nodeId2 Second node ID of the edge
     * @return True if edge was successfully enforced
     */
    bool enforceEdge(size_t nodeId1, size_t nodeId2);

    /**
     * @brief Split a constrained segment at its parametric midpoint
     *
     * Uses the parent edge geometry to find the correct midpoint on curved edges.
     * Splits adjacent triangles directly and restores Delaunay property via Lawson flipping.
     * Updates the constrained segments in MeshData2D (removes old, adds two new).
     *
     * @param segment The constrained segment to split
     * @param parentEdge The geometric edge the segment lies on
     * @return ID of the new midpoint node, or nullopt if split failed
     */
    std::optional<size_t> splitConstrainedSegment(
        const ConstrainedSegment2D& segment,
        const Geometry2D::IEdge2D& parentEdge);

    /**
     * @brief Remove all triangles outside the domain or in holes
     *
     * Removes all triangles that are not in the provided set of interior triangles.
     *
     * @param interiorTriangles Set of triangle IDs that should be kept (are inside the domain)
     * @return Vector of removed triangle IDs
     */
    std::vector<size_t> removeExteriorTriangles(const std::unordered_set<size_t>& interiorTriangles);

    /**
     * @brief Classify triangles as interior/exterior and remove exterior ones
     *
     * Convenience method combining classifyTrianglesInteriorExterior() and
     * removeExteriorTriangles() which are always called together.
     *
     * @return Vector of removed triangle IDs
     */
    std::vector<size_t> classifyAndRemoveExteriorTriangles();

    /**
     * @brief Get the mesh mutator for primitive operations
     */
    MeshMutator2D& getMutator() { return *mutator_; }
    const MeshMutator2D& getMutator() const { return *mutator_; }

    /**
     * @brief Get the mesh queries for read-only operations
     */
    MeshQueries2D& getQueries() { return queries_; }
    const MeshQueries2D& getQueries() const { return queries_; }

private:
    /**
     * @brief Split the triangles adjacent to an edge by inserting a midpoint node
     *
     * For each triangle sharing edge (edgeNode1, edgeNode2), replaces it with two
     * triangles connecting the new midpoint to the opposite vertex.
     *
     * @param edgeNode1 First node of the edge to split
     * @param edgeNode2 Second node of the edge to split
     * @param midNodeId ID of the already-inserted midpoint node
     * @return IDs of the newly created triangles
     */
    std::vector<size_t> splitTrianglesAtEdge(size_t edgeNode1, size_t edgeNode2, size_t midNodeId);

    /**
     * @brief Restore Delaunay property via Lawson edge flipping
     *
     * Checks edges of the given triangles and flips any that violate the
     * Delaunay criterion. Constrained edges are never flipped.
     *
     * @param newTriangleIds IDs of triangles to start checking from
     */
    void lawsonFlip(const std::vector<size_t>& newTriangleIds);

    MeshData2D& meshData_;
    PeriodicMeshData2D* periodicData_ = nullptr;
    MeshQueries2D queries_;
    std::unique_ptr<MeshMutator2D> mutator_;
    std::unique_ptr<ElementGeometry2D> geometry_;
};

} // namespace Meshing
