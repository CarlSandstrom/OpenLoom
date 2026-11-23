#pragma once

#include "Common/Types.h"
#include "Meshing/Core/3D/General/MeshQueries3D.h"
#include <array>
#include <optional>
#include <string>
#include <vector>

namespace Meshing
{

class MeshData3D;
class MeshMutator3D;

/**
 * @brief Mutation operations for 3D meshes
 *
 * Provides high-level mesh operations that modify the mesh, building upon MeshMutator3D.
 * Handles algorithms like Bowyer-Watson 3D insertion and cavity finding.
 * Uses MeshQueries3D for read-only query operations.
 */
class MeshOperations3D
{
public:
    MeshOperations3D(MeshData3D& meshData, MeshMutator3D& mutator);

    /**
     * @brief Create a bounding tetrahedron that contains all given points
     *
     * Creates a large tetrahedron that encloses all input points with margin.
     *
     * @param points The points that must be contained within the tetrahedron
     * @return Array of 4 node IDs for the bounding tetrahedron vertices
     */
    std::array<NodeId, 4> createBoundingTetrahedron(const std::vector<Point3D>& points);

    /**
     * @brief Insert a vertex using 3D Bowyer-Watson algorithm
     *
     * Finds conflicting tetrahedra, removes them to form a cavity,
     * and retriangulates with the new vertex. Maintains Delaunay property.
     *
     * @param point The 3D point to insert
     * @param geometryIds Optional geometry entity IDs this vertex belongs to
     * @param weight The vertex's regular-triangulation weight (0 for an
     * ordinary, unweighted vertex -- see Node3D::getWeight() and
     * RegularPredicates3D)
     * @return Node ID of the inserted vertex; nullopt, with the mesh untouched,
     * when no tetrahedron conflicts with the point -- it is hidden by a heavier
     * vertex, or lies outside the triangulation
     */
    std::optional<NodeId> insertVertexBowyerWatson(const Point3D& point,
                                                   const std::vector<std::string>& geometryIds = {},
                                                   double weight = 0.0);

    /**
     * @brief Insert a vertex using 3D Bowyer-Watson algorithm with pre-computed conflicting tetrahedra
     *
     * Overload for callers that have already computed the conflicting-tetrahedra set
     * (e.g. SurfaceDelaunayRefiner, which grows it locally and reads the cavity's faces
     * before calling this). Avoids a redundant findConflictingTetrahedra() scan.
     *
     * @param point The 3D point to insert
     * @param conflictingTetrahedra Pre-computed result of findConflictingTetrahedra(point, weight);
     * must not be empty (throws)
     * @param geometryIds Optional geometry entity IDs this vertex belongs to
     * @param weight The vertex's regular-triangulation weight (see the other overload)
     * @return Node ID of the inserted vertex
     */
    NodeId insertVertexBowyerWatson(const Point3D& point,
                                    std::vector<ElementId> conflictingTetrahedra,
                                    const std::vector<std::string>& geometryIds = {},
                                    double weight = 0.0);

    /**
     * @brief Raise an existing vertex's weight and retriangulate around it
     *
     * Removes cavity (RegularConflictRegion::findForWeightIncrease() for the
     * same vertex and weight), sets the weight, and fans the vertex onto the
     * cavity boundary. The caller makes sure no other vertex is hidden (each
     * vertex of the cavity lies on its boundary) and no boundary face is
     * coplanar with the vertex.
     *
     * @return The tetrahedra removed, as node arrays, so the caller can
     * restore them
     */
    std::vector<std::array<NodeId, 4>> raiseVertexWeight(NodeId vertexNodeId,
                                                         double newWeight,
                                                         const std::vector<ElementId>& cavity);

    /**
     * @brief Get the mesh mutator for primitive operations
     */
    MeshMutator3D& getMutator() { return mutator_; }
    const MeshMutator3D& getMutator() const { return mutator_; }

    /**
     * @brief Get the mesh queries for read-only operations
     */
    MeshQueries3D& getQueries() { return queries_; }
    const MeshQueries3D& getQueries() const { return queries_; }

private:
    MeshData3D& meshData_;
    MeshQueries3D queries_;
    MeshMutator3D& mutator_;

    /**
     * @brief Retriangulate the cavity with the new vertex
     *
     * Creates new tetrahedra connecting the vertex to the cavity boundary.
     *
     * @param vertexNodeId The newly inserted vertex
     * @param boundary The triangular faces forming the cavity boundary
     */
    void retriangulate(NodeId vertexNodeId,
                       const std::vector<std::array<NodeId, 3>>& boundary);

    /**
     * @brief Grow the conflicting-tetrahedra cavity through boundary faces
     * that are coplanar with the point being inserted
     *
     * Fanning a new tetrahedron from the inserted vertex to a cavity boundary
     * face that lies in the same plane as the vertex would produce a
     * zero-volume tetrahedron. Instead of leaving that face uncovered, pull
     * the tetrahedron on the other side of the face into the cavity too, so
     * retriangulate() can fan onto its far faces instead -- but only when
     * doing so keeps the cavity boundary a topological sphere (see
     * eulerCharacteristic() in the .cpp): growing indiscriminately can wrap
     * the cavity into a non-simply-connected shape that a single-point
     * vertex fan cannot validly cover (OPE-173). Any coplanar face this
     * declines to grow through is left on the returned set's boundary for
     * insertVertexBowyerWatson() to resolve via splitCoplanarBoundaryFace()
     * instead.
     *
     * @param point The point being inserted
     * @param conflicting The initial conflicting tetrahedra (by circumsphere test)
     * @return The grown set of conflicting tetrahedra
     */
    std::vector<ElementId> growCavityThroughCoplanarFaces(const Point3D& point,
                                                          std::vector<ElementId> conflicting) const;

    /**
     * @brief Build a single positively-oriented tetrahedron from a face and an apex
     *
     * Shared by retriangulate() and splitCoplanarBoundaryFace(): picks the
     * face winding (as stored) that gives the resulting TetrahedronElement a
     * positive signed volume under the usual (face..., apex) sign
     * convention, regardless of the face's own orientation.
     *
     * @param face The three face node IDs
     * @param apexNodeId The fourth (apex) node ID
     */
    void addOrientedTetrahedron(const std::array<NodeId, 3>& face, NodeId apexNodeId);

    /**
     * @brief Resolve a coplanar cavity boundary face growCavityThroughCoplanarFaces()
     * declined to grow through
     *
     * Splits the face into 3 sub-triangles around the newly inserted vertex
     * and fans each to both of the face's original apexes (one from the
     * removed side, one from the kept side) -- the standard pyramid
     * subdivision of the two tetrahedra that used to share this face, now
     * sharing the new vertex as an interior point of their common base
     * instead of the new vertex being fanned directly onto the (coplanar,
     * zero-volume) face.
     *
     * @param vertexNodeId The newly inserted vertex
     * @param face The coplanar boundary face being split
     * @param apex0 Apex of the tetrahedron on the removed (conflicting) side
     * @param apex1 Apex of the tetrahedron on the kept (neighbor) side
     */
    void splitCoplanarBoundaryFace(NodeId vertexNodeId,
                                   const std::array<NodeId, 3>& face,
                                   NodeId apex0,
                                   NodeId apex1);
};

} // namespace Meshing
