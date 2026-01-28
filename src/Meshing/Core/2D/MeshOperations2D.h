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
     * Inserts the new node via Bowyer-Watson and re-enforces both resulting segments.
     *
     * @param segment The constrained segment to split
     * @param parentEdge The geometric edge the segment lies on
     * @return Pair of new segments, or nullopt if split failed
     */
    std::optional<std::pair<ConstrainedSegment2D, ConstrainedSegment2D>> splitConstrainedSegment(
        const ConstrainedSegment2D& segment,
        const Geometry2D::IEdge2D& parentEdge);

    /**
     * @brief Classify triangles as interior/exterior using flood fill from constraint edges
     *
     * Uses mesh topology (constraint edges) to determine which triangles are inside
     * the domain vs outside or in holes. Finds a seed triangle farthest from constraints,
     * performs BFS flood fill respecting constraint boundaries, and removes unreached triangles.
     *
     * @param constrainedEdges Vector of constraint edges that form domain boundaries
     */
    void classifyTrianglesInteriorExterior(const std::vector<ConstrainedSegment2D>& constrainedEdges);

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
    MeshData2D& meshData_;
    MeshQueries2D queries_;
    std::unique_ptr<MeshMutator2D> mutator_;
    std::unique_ptr<ElementGeometry2D> geometry_;
};

} // namespace Meshing
