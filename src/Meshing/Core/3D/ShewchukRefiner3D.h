#pragma once

#include "GeometryStructures3D.h"
#include "Meshing/Interfaces/IQualityController3D.h"
#include "MeshingContext3D.h"
#include <vector>

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
 * Reference: Shewchuk, J.R. "Tetrahedral Mesh Generation by Delaunay Refinement"
 *            Proceedings of the Fourteenth Annual Symposium on Computational Geometry, 1998.
 */
class ShewchukRefiner3D
{
public:
    /**
     * @brief Construct a Shewchuk 3D refiner
     * @param context The meshing context containing mesh data and operations
     * @param qualityController Quality controller defining acceptable mesh quality
     * @param constrainedSubsegments List of constrained boundary edges
     * @param constrainedSubfacets List of constrained boundary faces
     */
    ShewchukRefiner3D(MeshingContext3D& context,
                      const IQualityController3D& qualityController,
                      std::vector<ConstrainedSubsegment3D>& constrainedSubsegments,
                      std::vector<ConstrainedSubfacet3D>& constrainedSubfacets);

    ~ShewchukRefiner3D();

    /**
     * @brief Run the refinement algorithm until quality goals are met
     *
     * Iteratively refines the mesh by:
     * 1. Splitting encroached subsegments (highest priority)
     * 2. Splitting encroached subfacets
     * 3. Inserting circumcenters of skinny tetrahedra (lowest priority)
     *
     * Continues until the quality controller is satisfied, the element limit
     * is reached, or no further progress can be made.
     */
    void refine();

private:
    MeshingContext3D* context_;
    const IQualityController3D* qualityController_;
    std::vector<ConstrainedSubsegment3D>& constrainedSubsegments_;
    std::vector<ConstrainedSubfacet3D>& constrainedSubfacets_;
};

} // namespace Meshing
