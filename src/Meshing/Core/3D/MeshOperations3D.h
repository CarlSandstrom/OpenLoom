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
