#pragma once

#include <array>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "Common/Types.h"

namespace Meshing
{

/**
 * @brief Constrained 2D Delaunay triangulation
 *
 * Provides unconstrained and constrained 2D Delaunay triangulation in parametric space.
 * Used for triangulating surfaces in the 3D mesher.
 *
 * Design:
 * - Works in 2D parametric space (u,v coordinates)
 * - Uses Bowyer-Watson incremental algorithm
 * - Supports constrained edges (for surface boundaries)
 * - Returns triangulation as list of node ID triplets
 *
 * Usage:
 * 1. Create instance with node ID to Point2D mapping
 * 2. Optionally add constraint edges
 * 3. Call triangulate() to get triangles
 */
class ConstrainedDelaunay2D
{
public:
    /**
     * @brief Construct 2D Delaunay triangulator
     * @param nodeCoords Map from node ID to 2D parametric coordinates
     */
    explicit ConstrainedDelaunay2D(const std::unordered_map<size_t, Point2D>& nodeCoords);

    /**
     * @brief Add a constraint edge that must appear in the triangulation
     * @param nodeId1 First node ID
     * @param nodeId2 Second node ID
     */
    void addConstraintEdge(size_t nodeId1, size_t nodeId2);

    /**
     * @brief Compute the triangulation
     * @return Vector of triangles, each as array of 3 node IDs
     */
    std::vector<std::array<size_t, 3>> triangulate();

private:
    struct Triangle
    {
        std::array<size_t, 3> nodes;

        Triangle(size_t n0, size_t n1, size_t n2) : nodes{n0, n1, n2} {}

        bool hasNode(size_t nodeId) const
        {
            return nodes[0] == nodeId || nodes[1] == nodeId || nodes[2] == nodeId;
        }

        bool containsAll(const std::array<size_t, 3>& nodeIds) const
        {
            return hasNode(nodeIds[0]) && hasNode(nodeIds[1]) && hasNode(nodeIds[2]);
        }

        std::array<size_t, 2> getEdge(size_t edgeIndex) const
        {
            switch (edgeIndex)
            {
            case 0: return {nodes[0], nodes[1]};
            case 1: return {nodes[1], nodes[2]};
            case 2: return {nodes[2], nodes[0]};
            default: return {nodes[0], nodes[1]};
            }
        }
    };

    struct CircumCircle
    {
        Point2D center;
        double radiusSquared;
    };

    // Node coordinates in 2D parametric space
    std::unordered_map<size_t, Point2D> nodeCoords_;

    // Constraint edges
    std::vector<std::pair<size_t, size_t>> constraintEdges_;

    // Active triangles during triangulation
    std::vector<Triangle> activeTriangles_;

    // Super triangle node IDs
    std::vector<size_t> superNodeIds_;

    /**
     * @brief Create super triangle that contains all points
     */
    void createSuperTriangle_();

    /**
     * @brief Insert a vertex into the triangulation
     */
    void insertVertex_(size_t nodeId);

    /**
     * @brief Find triangles whose circumcircle contains the point
     */
    std::vector<size_t> findConflictingTriangles_(const Point2D& point) const;

    /**
     * @brief Find the boundary of the cavity formed by conflicting triangles
     */
    std::vector<std::array<size_t, 2>> findCavityBoundary_(const std::vector<size_t>& conflictingIndices) const;

    /**
     * @brief Retriangulate the cavity with the new vertex
     */
    void retriangulate_(size_t vertexNodeId, const std::vector<std::array<size_t, 2>>& boundary);

    /**
     * @brief Remove super triangle and its associated triangles
     */
    void removeSuperTriangle_();

    /**
     * @brief Force a constraint edge to appear in the mesh
     */
    void forceEdge_(size_t nodeId1, size_t nodeId2);

    /**
     * @brief Check if edge exists in triangulation
     */
    bool edgeExists_(size_t nodeId1, size_t nodeId2) const;

    /**
     * @brief Find triangles that intersect with a constraint edge
     */
    std::vector<size_t> findIntersectingTriangles_(size_t nodeId1, size_t nodeId2) const;

    /**
     * @brief Compute circumcircle of a triangle
     */
    std::optional<CircumCircle> computeCircumcircle_(const Triangle& tri) const;

    /**
     * @brief Check if point is inside circumcircle
     */
    bool isPointInsideCircumcircle_(const CircumCircle& circle, const Point2D& point) const;

    /**
     * @brief Check if two 2D segments intersect
     */
    bool segmentsIntersect_(const Point2D& a1, const Point2D& a2,
                            const Point2D& b1, const Point2D& b2) const;

    /**
     * @brief Create ordered edge key for lookups
     */
    static std::pair<size_t, size_t> makeEdgeKey_(size_t a, size_t b)
    {
        return (a < b) ? std::make_pair(a, b) : std::make_pair(b, a);
    }
};

} // namespace Meshing
