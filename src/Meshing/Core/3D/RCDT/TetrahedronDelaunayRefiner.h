#pragma once

#include "Common/Types.h"
#include "Meshing/Core/3D/RCDT/RestrictedFaceTypes.h"
#include "Meshing/Core/3D/RCDT/SurfaceDelaunayRefiner.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"

#include <cstddef>
#include <unordered_set>

namespace Topology3D
{
class Topology3D;
} // namespace Topology3D

namespace Meshing
{
class MeshingContext3D;
class TetrahedronElement;
} // namespace Meshing

namespace Meshing
{

/**
 * @brief Volume refinement as CGAL Mesh_3 does it (Refine_cells_3): the level
 * below SurfaceDelaunayRefiner, used by RCDTMesher::meshVolume().
 *
 *  - A tetrahedron is refined when it lies inside the domain and its weighted
 *    circumradius-to-shortest-edge ratio exceeds
 *    tetrahedronCircumradiusToShortestEdgeRatio. Inside is decided by
 *    AmbientTetrahedronClassifier's flood fill from the restricted facets,
 *    recomputed once per round; CGAL asks the domain oracle at the weighted
 *    circumcentre instead.
 *  - The refinement point is the weighted circumcentre (orthocentre).
 *  - The surface always comes first: after every insertion the facet level
 *    refines until no facet is bad, and a point whose conflict region holds a
 *    restricted facet encroached by it is not inserted -- that facet is
 *    refined instead.
 *  - A point hidden by, or coincident with, an existing vertex is refused,
 *    and its tetrahedron left as it is.
 *
 * No sliver perturbation or exudation. Terminates on a round with no
 * insertion or at the facet level's insertion cap, which counts both levels.
 */
class TetrahedronDelaunayRefiner
{
public:
    TetrahedronDelaunayRefiner(MeshingContext3D& context,
                               const Topology3D::Topology3D& topology,
                               const SurfaceMesh3DQualitySettings& settings,
                               double minimumEdgeLength);

    /// Refines the surface, then the volume.
    void refine();

    /// The restricted facets of the level below; valid once refine() has run.
    RestrictedFaceMap getRestrictedFaces() const { return surfaceRefiner_.getRestrictedFaces(); }

    /// The restriction test of the level below.
    const WeightedDualRestriction& getRestriction() const { return surfaceRefiner_.getRestriction(); }

private:
    MeshingContext3D* context_;
    SurfaceDelaunayRefiner surfaceRefiner_;
    double ratioBound_;
    std::unordered_set<ElementId> unrefinable_;
    std::size_t insertionCount_ = 0;
    std::size_t encroachmentCount_ = 0;

    bool refineRound();
    bool refineTetrahedron(ElementId elementId, const TetrahedronElement& tetrahedron);
};

} // namespace Meshing
