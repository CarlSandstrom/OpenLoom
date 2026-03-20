#pragma once

#include "Common/Types.h"
#include "Meshing/Core/2D/DiscretizationResult2D.h"
#include "Meshing/Core/2D/MeshingContext2D.h"
#include "Meshing/Core/3D/General/GeometryStructures3D.h"
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace Geometry3D
{
class ISurface3D;
class GeometryCollection3D;
} // namespace Geometry3D

namespace Topology3D
{
class Surface3D;
class Topology3D;
} // namespace Topology3D

namespace Meshing
{

/**
 * @brief Manages 2D Delaunay triangulation in parametric space for a single 3D surface
 *
 * Each facet (surface) maintains its own independent 2D triangulation that is
 * updated when vertices are inserted on the facet during refinement. This class
 * maintains bidirectional mapping between 3D mesh node IDs and 2D facet node IDs,
 * and an edge-to-node-sequence map used for building the surface-aware TwinManager.
 *
 * Per Shewchuk's algorithm, facet triangulations define what subfacets *should*
 * exist in the final mesh. They are maintained separately from the 3D tetrahedralization.
 */
class FacetTriangulation
{
public:
    /**
     * @brief Construct facet triangulation for a surface
     * @param surface The 3D surface geometry
     * @param topoSurface The surface topology
     * @param fullTopology Full 3D topology for edge/corner lookups
     * @param fullGeometry Full 3D geometry collection
     */
    FacetTriangulation(const Geometry3D::ISurface3D& surface,
                       const Topology3D::Surface3D& topoSurface,
                       const Topology3D::Topology3D& fullTopology,
                       const Geometry3D::GeometryCollection3D& fullGeometry);

    ~FacetTriangulation();

    // Prevent copying
    FacetTriangulation(const FacetTriangulation&) = delete;
    FacetTriangulation& operator=(const FacetTriangulation&) = delete;

    // Allow moving
    FacetTriangulation(FacetTriangulation&&) noexcept;
    FacetTriangulation& operator=(FacetTriangulation&&) noexcept;

    /**
     * @brief Initialize triangulation using a constrained 2D Delaunay triangulation.
     *
     * Runs ConstrainedDelaunay2D on the UV-space discretization: registers boundary
     * edge segments as constraints, enforces them, and removes exterior triangles.
     * After this call, MeshData2D contains a valid constrained triangulation ready
     * for ShewchukRefiner2D.
     *
     * Also builds the edge-to-node-sequence map used by FacetTriangulationManager
     * when constructing the surface-aware TwinManager.
     *
     * @param discretization2D  UV-space discretization for this face (local point indices)
     * @param localIndexToNode3DId  Maps each local discretization2D point index to its global 3D node ID
     */
    void initialize(DiscretizationResult2D discretization2D,
                    std::vector<size_t> localIndexToNode3DId);

    /**
     * @brief Get the surface ID
     */
    const std::string& getSurfaceId() const { return surfaceId_; }

    /**
     * @brief Get all subfacets (triangles) from the 2D triangulation
     *
     * Converts each 2D triangle to a ConstrainedSubfacet3D using the
     * 2D->3D node ID mapping.
     *
     * @return Vector of ConstrainedSubfacet3D with 3D node IDs
     */
    std::vector<ConstrainedSubfacet3D> getSubfacets() const;

    /**
     * @brief Insert a vertex into the facet triangulation
     *
     * Used during refinement when splitting subfacets. The vertex is
     * inserted into the 2D triangulation and the mappings are updated.
     *
     * @param node3DId The 3D mesh node ID
     * @param uvCoords The (u,v) parametric coordinates
     * @return true if insertion succeeded
     */
    bool insertVertex(size_t node3DId, const Point2D& uvCoords);

    /**
     * @brief Get the 2D node ID corresponding to a 3D node ID
     * @param node3DId The 3D mesh node ID
     * @return The 2D facet node ID, or std::nullopt if not found
     */
    std::optional<size_t> get2DNodeId(size_t node3DId) const;

    /**
     * @brief Get the 3D node ID corresponding to a 2D node ID
     * @param node2DId The 2D facet node ID
     * @return The 3D mesh node ID, or std::nullopt if not found
     */
    std::optional<size_t> get3DNodeId(size_t node2DId) const;

    /**
     * @brief Get the ordered sequence of 2D node IDs for an edge.
     *
     * Used by FacetTriangulationManager::buildTwinManager() to register
     * surface-aware twin segment pairs by local 2D IDs.
     *
     * @param edgeId The edge ID to look up
     * @return Pointer to the node sequence, or nullptr if the edge is not on this face
     */
    const std::vector<size_t>* getEdgeNodeSequence(const std::string& edgeId) const;

    /**
     * @brief Get the ordered sequence of 3D node IDs for an edge.
     *
     * Converts the 2D node sequence for @p edgeId to 3D node IDs using the
     * internal 2D→3D mapping.  Returns an empty vector if the edge is not on
     * this face.
     *
     * @param edgeId The edge ID to look up
     * @return Ordered 3D node IDs along the edge, or empty if not on this face
     */
    std::vector<size_t> getEdge3DNodeIds(const std::string& edgeId) const;

    /**
     * @brief Get 3D node sequences for all edges on this face.
     *
     * Returns a map from edge ID to ordered 3D node IDs for every edge whose
     * node sequence is known to this face.  Used by FacetTriangulationManager
     * to assemble SurfaceMesh3D::edgeNodeIds after refinement.
     */
    std::map<std::string, std::vector<size_t>> getAllEdge3DNodeIds() const;

    /**
     * @brief Register a known 2D↔3D node mapping.
     *
     * Used by SurfaceMeshingContext3D when a boundary split is propagated to a
     * twin surface — both the original and twin 2D nodes map to the same 3D node.
     * Calling this before resolveRefinementNodes() prevents duplicate 3D IDs.
     *
     * @param node2DId  2D node ID (in this facet's MeshData2D)
     * @param node3DId  3D node ID to associate with it
     */
    void registerNode(size_t node2DId, size_t node3DId);

    /**
     * @brief Update the edge node sequence after a boundary segment split.
     *
     * When a boundary segment (nodeId1, nodeId2) is split during refinement,
     * this method finds the edge whose sequence contains that adjacent pair and
     * inserts midNodeId between them. This keeps edgeIdToNode2DSeq_ current
     * throughout refinement so it accurately reflects the boundary discretization
     * at every stage (required by S3.3 twin consistency verification).
     *
     * Must be called by SurfaceMeshingContext3D after every boundary split — both
     * on the face where the split originates and on the twin face where it is applied.
     *
     * @param nodeId1   First endpoint of the split segment (2D node ID)
     * @param nodeId2   Second endpoint of the split segment (2D node ID)
     * @param midNodeId The new node inserted between nodeId1 and nodeId2 (2D node ID)
     */
    void updateEdgeNodeAfterSplit(size_t nodeId1, size_t nodeId2, size_t midNodeId);

    /**
     * @brief Map any 2D refinement nodes (inserted during Shewchuk refinement) to new 3D node IDs.
     *
     * After refinement, the 2D mesh may contain nodes with no 3D peer (ShewchukRefiner2D
     * inserts them without creating a matching 3D node). This method assigns IDs starting at
     * nextNode3DId (incremented for each new node), registers bidirectional mappings, and
     * returns the 3D positions obtained by lifting the UV coordinates onto the surface.
     *
     * @param nextNode3DId  Next available 3D node ID (incremented in-place per new node)
     * @return 3D positions of the newly registered nodes, in ID order
     */
    std::vector<Point3D> resolveRefinementNodes(size_t& nextNode3DId);

    /**
     * @brief Access the underlying 2D mesh context
     */
    MeshingContext2D& getContext() { return *context_; }
    const MeshingContext2D& getContext() const { return *context_; }

private:
    std::string surfaceId_;
    const Geometry3D::ISurface3D* surface_;

    std::unique_ptr<MeshingContext2D> context_;

    // Bidirectional mapping between 3D mesh node IDs and 2D facet node IDs
    std::map<size_t, size_t> node3DTo2DMap_;
    std::map<size_t, size_t> node2DTo3DMap_;

    // Maps edge ID → ordered sequence of 2D node IDs along that edge.
    // Built during initialize() and used by FacetTriangulationManager::buildTwinManager().
    std::map<std::string, std::vector<size_t>> edgeIdToNode2DSeq_;
};

} // namespace Meshing
