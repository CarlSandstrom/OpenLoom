#pragma once

#include "Meshing/Connectivity/FaceKey.h"
#include "Meshing/Core/3D/RCDT/RestrictedFaceTypes.h"
#include "Meshing/Core/3D/RCDT/SurfaceFacetCriteria.h"
#include "Meshing/Core/3D/RCDT/WeightedDualRestriction.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"

#include <cstddef>
#include <memory>
#include <optional>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace Topology3D
{
class Topology3D;
} // namespace Topology3D

namespace Meshing
{
class MeshConnectivity;
class MeshingContext3D;
} // namespace Meshing

namespace Meshing
{

/**
 * @brief Surface refinement as CGAL Mesh_3 does it (Refine_facets_3), on the
 * weighted tetrahedralization RCDTMesher seeds: an alternative to the
 * RestrictedTriangulation + RCDTRefiner path, for comparison (OPE-186).
 *
 * The design is CGAL's as a whole, because its pieces were each measured to
 * fail when ported one at a time into the existing refiner:
 *
 *  - Restriction by the weighted dual edge alone (WeightedDualRestriction).
 *  - CGAL's three facet criteria (SurfaceFacetCriteria).
 *  - Worst facet first: ordered by the first criterion that fails, then by
 *    how badly.
 *  - The refinement point is the facet's surface Delaunay ball centre.
 *  - The only refusal is CGAL's: a point is not inserted when neither
 *    tetrahedron beside the facet conflicts with it. No size floor, proximity
 *    guard or protecting-ball check; protection is fixed after seeding and
 *    curves are never split.
 *  - No post-processing: the restricted facets are the surface.
 *
 * Terminates on an empty queue or after maxRefinementIterations insertions,
 * the counterpart of CGAL's maximal_number_of_vertices.
 */
class SurfaceDelaunayRefiner
{
public:
    SurfaceDelaunayRefiner(MeshingContext3D& context,
                           const Topology3D::Topology3D& topology,
                           const SurfaceMesh3DQualitySettings& settings,
                           double minimumEdgeLength);
    ~SurfaceDelaunayRefiner();
    SurfaceDelaunayRefiner(const SurfaceDelaunayRefiner&) = delete;
    SurfaceDelaunayRefiner& operator=(const SurfaceDelaunayRefiner&) = delete;

    void refine();

    /// The restricted facets, each against the surface it was restricted to.
    RestrictedFaceMap getRestrictedFaces() const;

    std::size_t getInsertionCount() const { return insertionCount_; }

    /// Facets whose refinement point did not conflict with either of their
    /// tetrahedra, left as they are (see the class doc).
    std::size_t getDroppedCount() const { return droppedFaces_.size(); }

private:
    using Badness = SurfaceFacetCriteria::Badness;

    MeshingContext3D* context_;
    SurfaceMesh3DQualitySettings settings_;
    WeightedDualRestriction restriction_;
    SurfaceFacetCriteria criteria_;

    /// Rebuilt after every insertion; the tetrahedralization changes under it.
    std::unique_ptr<MeshConnectivity> connectivity_;

    std::unordered_map<FaceKey, RestrictedFacet, FaceKeyHash> facets_;
    std::unordered_map<FaceKey, Badness, FaceKeyHash> badness_;
    std::set<std::pair<Badness, FaceKey>> queue_;
    std::unordered_set<FaceKey, FaceKeyHash> droppedFaces_;
    std::size_t insertionCount_ = 0;

    void classifyAllFaces();
    void reclassify(const FaceKey& face);
    void forget(const FaceKey& face);
    bool refineWorst();
};

} // namespace Meshing
