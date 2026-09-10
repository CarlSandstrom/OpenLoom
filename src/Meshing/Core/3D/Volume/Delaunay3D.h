#pragma once

#include "Common/Types.h"
#include <array>
#include <map>
#include <string>
#include <vector>

namespace Meshing
{

class MeshOperations3D;

/// What one triangulate() call produces, beyond the tetrahedra it writes
/// into the MeshOperations3D it was given.
struct Delaunay3DResult
{
    /// Mapping from input point index to mesh node ID.
    std::map<size_t, size_t> pointIndexToNodeIdMap;

    /// The 4 node IDs of the bounding tetrahedron left in the mesh by
    /// triangulate(). The caller owns removing them when appropriate.
    std::array<size_t, 4> boundingNodeIds{};
};

/**
 * @brief Simple Delaunay tetrahedralization in 3D
 *
 * This class provides a wrapper around the Bowyer-Watson algorithm for
 * creating a Delaunay tetrahedralization from a set of 3D points.
 * Mirrors the API of Delaunay2D for consistency.
 *
 * Operates through a caller-supplied MeshOperations3D rather than
 * constructing its own: a fresh MeshOperations3D's node-ID counter is only
 * initialized from whatever nodes already exist in the mesh at construction
 * time, so a second, independently-constructed instance operating on the
 * same MeshData3D concurrently (e.g. the caller's own long-lived
 * MeshOperations3D used for later refinement) would not see the nodes this
 * class adds and could reissue their IDs.
 *
 * The bounding (super-)tetrahedron used internally to seed the incremental
 * insertion is intentionally left in the mesh after triangulate() returns.
 * Removing it immediately would mean every subsequently-inserted point (e.g.
 * during later refinement) that ends up on the true convex hull has cavity
 * boundary faces with no neighbor to fall back on if they turn out to be
 * coplanar with the new vertex. Keeping it in place guarantees a neighbor is
 * always reachable; the caller is responsible for removing it (via
 * Delaunay3DResult::boundingNodeIds and
 * MeshOperations3D::removeBoundingTetrahedron()) once it is safe to do so.
 */
class Delaunay3D
{
public:
    /**
     * @brief Perform the Delaunay tetrahedralization. The bounding
     * tetrahedron is left in the mesh -- see class documentation.
     * @param operations The mesh operations to perform insertions through --
     * must be the same instance the caller uses for any later mutation of
     * the same mesh (see class documentation)
     * @param points Vector of Point3D representing the input vertices
     * @param geometryIds Geometry IDs for each point (corner/edge/surface IDs)
     * @param pointWeights Regular-triangulation weight for each point (0 --
     * an ordinary, unweighted point -- if index-absent or the vector is
     * left empty; see Node3D::getWeight() and RegularPredicates3D, OPE-176)
     */
    static Delaunay3DResult triangulate(MeshOperations3D& operations,
                                        const std::vector<Point3D>& points,
                                        const std::vector<std::vector<std::string>>& geometryIds = {},
                                        const std::vector<double>& pointWeights = {});
};

} // namespace Meshing
