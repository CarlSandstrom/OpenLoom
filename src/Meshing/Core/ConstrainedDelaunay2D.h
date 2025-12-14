#pragma once

#include "Common/Types.h"
#include "Meshing/Data/MeshData2D.h"
#include "Meshing/Data/MeshData3D.h"
#include "Meshing/Data/TriangleElement.h"
#include <array>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace Meshing
{

class MeshingContext2D;
class MeshOperations2D;

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
 * Usage (with MeshingContext2D):
 * 1. Create instance with MeshingContext2D
 * 2. Call generateConstrained() to create mesh from topology/geometry
 *
 * Usage (standalone):
 * 1. Create instance with node ID to Point2D mapping
 * 2. Optionally add constraint edges
 * 3. Call triangulate() to get triangles
 */
class ConstrainedDelaunay2D
{
public:
    /**
     * @brief Construct 2D Delaunay triangulator with MeshingContext2D
     *
     * This constructor enables the generateConstrained() workflow,
     * similar to ConstrainedDelaunay3D.
     *
     * @param context The 2D meshing context containing geometry and topology
     */
    explicit ConstrainedDelaunay2D(MeshingContext2D& context);

    /**
     * @brief Construct 2D Delaunay triangulator with raw coordinates
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

    /**
     * @brief Generate constrained mesh from topology (requires MeshingContext2D)
     *
     * Process:
     * 1. Insert corner nodes from topology
     * 2. Sample and insert edge nodes
     * 3. Create initial Delaunay triangulation
     * 4. Force each constraint edge into mesh
     *
     * @param samplesPerEdge Number of sample points per boundary edge
     */
    void generateConstrained(size_t samplesPerEdge = 10);

    /**
     * @brief Get the 2D mesh data (requires MeshingContext2D constructor)
     */
    MeshData2D& getMeshData2D();
    const MeshData2D& getMeshData2D() const;

    /**
     * @brief Get mesh data as 3D (for backward compatibility)
     * @deprecated Use getMeshData2D() instead
     */
    MeshData getMeshData() const;

private:
    struct CircumCircle
    {
        Point2D center;
        double radiusSquared;
    };

    // Context (optional - for generateConstrained workflow)
    MeshingContext2D* context_ = nullptr;
    MeshData2D* meshData2D_ = nullptr;
    MeshOperations2D* operations_ = nullptr;

    // Maps topology IDs to mesh node IDs (for generateConstrained)
    std::unordered_map<std::string, size_t> topologyToNodeId_;

    // Node coordinates in 2D parametric space
    std::unordered_map<size_t, Point2D> nodeCoords_;

    // Constraint edges
    std::vector<std::pair<size_t, size_t>> constraintEdges_;

    // Active triangles during triangulation
    std::vector<TriangleElement> activeTriangles_;

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
    std::optional<CircumCircle> computeCircumcircle_(const TriangleElement& tri) const;

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

    // Helper methods for generateConstrained workflow
    void insertCornerNodes_();
    void insertEdgeNodes_(size_t samplesPerEdge);
    void buildTriangulation_();
    void recoverConstraintEdges_();
    void storeResultsInMeshData_();
};

} // namespace Meshing
