#pragma once

#include "Common/Types.h"
#include "Meshing/Core/2D/Computer2D.h"
#include "Meshing/Core/2D/GeometryStructures2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include <array>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
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
 * @brief General mesh operations for 2D meshes
 *
 * Provides high-level mesh operations that build upon MeshMutator2D.
 * Handles algorithms like Bowyer-Watson, cavity finding, and triangle intersection.
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
     * @brief Insert a vertex using Bowyer-Watson algorithm
     * @param nodeId The node ID to insert
     * @param nodeCoords Map of all node coordinates
     * @param activeTriangles In/out vector of active triangles
     */
    void insertVertexBowyerWatson(size_t nodeId,
                                  const std::unordered_map<size_t, Point2D>& nodeCoords,
                                  std::vector<TriangleElement>& activeTriangles) const; // TODO: Remove this?
    size_t insertVertexBowyerWatson(const Point2D& point);
    size_t insertBoundaryVertexBowyerWatson(const Point2D& point, double edgeParameter, const std::string& geometryId);

    /**
     * @brief Find triangles whose circumcircle contains the point
     * @param point The point to test
     * @return Ids of conflicting triangles
     */
    std::vector<size_t> findConflictingTriangles(const Point2D& point) const;

    /**
     * @brief Find the boundary of the cavity formed by conflicting triangles
     * @param conflictingIndices Indices of conflicting triangles
     * @return Boundary edges of the cavity
     */
    std::vector<std::array<size_t, 2>> findCavityBoundary(const std::vector<size_t>& conflictingIndices) const;

    bool removeTrianglesContainingNode(size_t nodeId);

    /**
     * @brief Find triangles that intersect with an edge
     * @param nodeId1 First node ID of the edge
     * @param nodeId2 Second node ID of the edge
     * @return Indices of intersecting triangles
     */
    std::vector<size_t> findIntersectingTriangles(size_t nodeId1, size_t nodeId2) const;

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
     * @brief Extract constrained edges from topology as constrained segments
     *
     * Converts topology edge definitions into ConstrainedSegment2D using
     * the point-to-node mapping from triangulation. If edges have been discretized
     * into multiple segments, creates constrained segments for each.
     *
     * @param topology The topology containing edge definitions
     * @param cornerIdToPointIndexMap Maps corner IDs to point array indices
     * @param pointIndexToNodeIdMap Maps point array indices to mesh node IDs
     * @param edgeIdToPointIndicesMap Maps edge IDs to ordered point indices (including intermediate points)
     * @return Vector of constrained segments
     */
    std::vector<ConstrainedSegment2D> extractConstrainedEdges(
        const Topology2D::Topology2D& topology,
        const std::map<std::string, size_t>& cornerIdToPointIndexMap,
        const std::map<size_t, size_t>& pointIndexToNodeIdMap,
        const std::map<std::string, std::vector<size_t>>& edgeIdToPointIndicesMap) const;

    /**
     * @brief Get the mesh mutator for primitive operations
     */
    MeshMutator2D& getMutator() { return *mutator_; }
    const MeshMutator2D& getMutator() const { return *mutator_; }

private:
    MeshData2D& meshData_;
    std::unique_ptr<MeshMutator2D> mutator_;
    std::unique_ptr<Computer2D> computer_;

    /**
     * @brief Retriangulate the cavity with the new vertex
     */
    void retriangulate(size_t vertexNodeId,
                       const std::vector<std::array<size_t, 2>>& boundary,
                       std::vector<TriangleElement>& activeTriangles) const;

    /**
     * @brief Check if two 2D segments intersect
     */
    bool segmentsIntersect(const Point2D& a1, const Point2D& a2,
                           const Point2D& b1, const Point2D& b2) const;

    /**
     * @brief Create ordered edge key for lookups
     */
    static std::pair<size_t, size_t> makeEdgeKey(size_t a, size_t b)
    {
        return (a < b) ? std::make_pair(a, b) : std::make_pair(b, a);
    }
};

} // namespace Meshing
