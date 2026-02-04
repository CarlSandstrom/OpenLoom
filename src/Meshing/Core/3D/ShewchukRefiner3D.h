#pragma once

#include "Meshing/Interfaces/IQualityController3D.h"
#include "MeshingContext3D.h"
#include <unordered_set>

namespace Geometry3D
{
class GeometryCollection3D;
class IEdge3D;
class ISurface3D;
} // namespace Geometry3D

namespace Meshing
{

/**
 * @brief Implements Shewchuk's 3D Delaunay refinement algorithm for tetrahedral meshes
 *
 * This class refines a 3D Delaunay mesh following the algorithm described in:
 * "Tetrahedral Mesh Generation by Delaunay Refinement" by Jonathan Richard Shewchuk
 *
 * The algorithm uses three prioritized operations:
 *
 * Priority 1 - Split encroached subsegments:
 *   A subsegment is encroached if a vertex (other than its endpoints) lies inside
 *   its diametral sphere. Split at the midpoint.
 *
 * Priority 2 - Split encroached subfacets:
 *   A subfacet is encroached if a non-coplanar vertex lies inside its equatorial
 *   sphere. Split at the circumcenter, but if that would encroach any subsegments,
 *   split those first.
 *
 * Priority 3 - Split skinny tetrahedra:
 *   A tetrahedron is "skinny" if its circumradius-to-shortest-edge ratio exceeds
 *   bound B (typically B > 2). Split at the circumcenter, but if that would
 *   encroach any subsegments or subfacets, split those first.
 *
 * Constraints (subsegments, subfacets) are read from MeshData3D via the context.
 *
 * Reference: Shewchuk, J.R. "Tetrahedral Mesh Generation by Delaunay Refinement"
 *            Proceedings of the Fourteenth Annual Symposium on Computational Geometry, 1998.
 */
class ShewchukRefiner3D
{
public:
    /**
     * @brief Construct a Shewchuk 3D refiner
     * @param context The meshing context containing mesh data, operations, and constraints
     * @param qualityController Quality controller defining acceptable mesh quality
     */
    ShewchukRefiner3D(MeshingContext3D& context,
                      const IQualityController3D& qualityController);

    ~ShewchukRefiner3D();

    /**
     * @brief Run the refinement algorithm until quality goals are met
     */
    void refine();

private:
    MeshingContext3D* context_;
    const IQualityController3D* qualityController_;

    // Track tetrahedra that cannot be refined
    std::unordered_set<size_t> unrefinableTetrahedra_;

    /**
     * @brief Perform a single refinement step
     * @return true if a refinement was performed
     */
    bool refineStep();

    // ========== Priority 3: Skinny tetrahedra handling ==========

    /**
     * @brief Handle a skinny tetrahedron by inserting its circumcenter
     * @param tetId ID of the tetrahedron to refine
     * @return true if refinement was successful
     */
    bool handleSkinnyTetrahedron(size_t tetId);
};

} // namespace Meshing
