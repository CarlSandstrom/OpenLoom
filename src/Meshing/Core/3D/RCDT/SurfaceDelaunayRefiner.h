#pragma once

#include "Common/Types.h"
#include "Meshing/Connectivity/FaceKey.h"
#include "Meshing/Core/3D/RCDT/RestrictedFaceTypes.h"
#include "Meshing/Core/3D/RCDT/SurfaceFacetCriteria.h"
#include "Meshing/Core/3D/RCDT/WeightedDualRestriction.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"

#include <cstddef>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

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
 * weighted tetrahedralization RCDTMesher seeds; the default in place of the
 * RestrictedTriangulation + RCDTRefiner path (OPE-186). When meshing a volume,
 * TetrahedronDelaunayRefiner drives it as the level above its own.
 *
 * The design is CGAL's as a whole, because its pieces were each measured to
 * fail when ported one at a time into the existing refiner:
 *
 *  - Restriction by the weighted dual edge alone (WeightedDualRestriction).
 *  - CGAL's three facet criteria (SurfaceFacetCriteria).
 *  - Worst facet first: ordered by the first criterion that fails, then by
 *    how badly.
 *  - The refinement point is the facet's surface Delaunay ball centre.
 *  - CGAL's refusals only: a point is not inserted when neither tetrahedron
 *    beside the facet conflicts with it, or when an existing vertex hides it
 *    (it lies in that vertex's protecting ball) or coincides with it. No size
 *    floor or proximity guard; protection is fixed after seeding and curves
 *    are never split.
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

    /// Classifies every face, then refines until no facet is bad.
    void refine();

    /// Refines until no facet is bad, after refine() has run: the facet level
    /// catching up after TetrahedronDelaunayRefiner inserts a point.
    void refineQueued();

    /// Inserts facet's surface Delaunay ball centre unless CGAL's refusals
    /// apply, in which case the facet is dropped. True if a point was
    /// inserted. Also used for a facet that is not bad but encroached.
    bool refineFacet(const FaceKey& face);

    /// Inserts point into its conflict region and reclassifies every face the
    /// insertion created or destroyed.
    void insert(const Point3D& point, std::vector<std::size_t> conflictRegion, std::vector<std::string> geometryIds);

    /// A restricted facet among the conflict region's faces whose surface
    /// Delaunay ball contains point: CGAL's encroachment, which makes the
    /// tetrahedron level refine that facet instead of inserting point. Every
    /// restricted facet the insertion would destroy is encroached.
    std::optional<FaceKey> findEncroachedFacet(const Point3D& point,
                                               const std::vector<std::size_t>& conflictRegion) const;

    bool hasReachedInsertionCap() const;

    /// Current after every insertion; valid once refine() has run.
    const MeshConnectivity& getConnectivity() const { return *connectivity_; }

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
    std::size_t insertionCount_ = 0; // facet and tetrahedron insertions alike

    void classifyAllFaces();
    void reclassify(const FaceKey& face);
    void forget(const FaceKey& face);
    void drop(const FaceKey& face);
    bool refineWorst();
};

} // namespace Meshing
