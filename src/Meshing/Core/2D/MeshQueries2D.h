#pragma once

#include "Common/Types.h"
#include "Meshing/Core/2D/GeometryStructures2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include <array>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
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

/**
 * @brief Query operations for 2D meshes (read-only)
 *
 * Provides methods for querying mesh data without modifying it.
 * Handles algorithms like finding conflicting triangles, cavity boundaries,
 * and encroached segments.
 */
class MeshQueries2D
{
public:
    using EdgeKey = std::pair<size_t, size_t>;

    struct EdgeKeyHash
    {
        std::size_t operator()(const EdgeKey& key) const
        {
            return std::hash<size_t>{}(key.first) ^ (std::hash<size_t>{}(key.second) << 1);
        }
    };

    using EdgeToTrianglesMap = std::unordered_map<EdgeKey, std::vector<size_t>, EdgeKeyHash>;

    /**
     * @brief Construct mesh queries with mesh data
     * @param meshData Reference to the 2D mesh data (read-only)
     */
    explicit MeshQueries2D(const MeshData2D& meshData);

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

    /**
     * @brief Find triangles that intersect with an edge
     * @param nodeId1 First node ID of the edge
     * @param nodeId2 Second node ID of the edge
     * @return Indices of intersecting triangles
     */
    std::vector<size_t> findIntersectingTriangles(size_t nodeId1, size_t nodeId2) const;

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
     * @brief Find all segments encroached by existing mesh vertices
     *
     * A segment is encroached if any mesh vertex (other than the segment endpoints)
     * lies within the segment's diametral circle.
     *
     * @return Vector of encroached constrained segments
     */
    std::vector<ConstrainedSegment2D> findEncroachedSegments() const;

    /**
     * @brief Check if a point would encroach any constrained segments
     *
     * A point encroaches a segment if it lies within the segment's diametral circle.
     *
     * @param point The candidate point to check
     * @return Vector of segments that would be encroached by the point
     */
    std::vector<ConstrainedSegment2D> findSegmentsEncroachedByPoint(const Point2D& point) const;

    /**
     * @brief Check if two 2D segments intersect
     * @param a1 First point of first segment
     * @param a2 Second point of first segment
     * @param b1 First point of second segment
     * @param b2 Second point of second segment
     * @return True if segments intersect
     */
    bool segmentsIntersect(const Point2D& a1, const Point2D& a2,
                           const Point2D& b1, const Point2D& b2) const;

    /**
     * @brief Create ordered edge key for lookups
     * @param a First node ID
     * @param b Second node ID
     * @return Ordered pair with smaller ID first
     */
    static std::pair<size_t, size_t> makeEdgeKey(size_t a, size_t b)
    {
        return (a < b) ? std::make_pair(a, b) : std::make_pair(b, a);
    }

    /**
     * @brief Find the 1 or 2 triangles sharing a given edge
     *
     * @param nodeId1 First node ID of the edge
     * @param nodeId2 Second node ID of the edge
     * @return Vector of triangle IDs adjacent to the edge (0, 1, or 2)
     */
    std::vector<size_t> findTrianglesAdjacentToEdge(size_t nodeId1, size_t nodeId2) const;

    /**
     * @brief Find the common geometry ID shared by two nodes
     * @param nodeId1 First node ID
     * @param nodeId2 Second node ID
     * @return The common geometry ID, or std::nullopt if none found
     */
    std::optional<std::string> findCommonGeometryId(size_t nodeId1, size_t nodeId2) const;

    /**
     * @brief Check if an edge is a boundary constraint edge
     * @param edgeId Edge node IDs
     * @return True if the edge is a boundary constraint edge
     */
    bool isBoundaryConstraintEdge(const std::array<size_t, 2>& edgeId) const;

    /**
     * @brief Build an edge-to-triangle adjacency map
     * @return Map from edge key to adjacent triangle IDs
     */
    EdgeToTrianglesMap buildEdgeToTrianglesMap() const;

    /**
     * @brief Test if a point is inside the domain using ray casting
     *
     * Casts a horizontal ray in the +X direction from the point and counts
     * crossings with boundary constraint segments. Odd = inside, even = outside.
     *
     * @param point The point to test
     * @return True if the point is inside the domain boundary
     */
    bool isPointInsideDomain(const Point2D& point) const;

    /**
     * @brief Classify triangles as interior using ray casting and flood fill
     *
     * Uses ray casting to find a seed triangle guaranteed to be inside the
     * domain, then performs BFS flood fill respecting constraint boundaries
     * to find all interior triangles.
     *
     * @return Set of triangle IDs that are inside the domain
     */
    std::unordered_set<size_t> classifyTrianglesInteriorExterior() const;

private:
    const MeshData2D& meshData_;
};

} // namespace Meshing
