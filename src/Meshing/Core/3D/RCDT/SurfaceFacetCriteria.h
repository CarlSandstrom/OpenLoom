#pragma once

#include "Meshing/Connectivity/FaceKey.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"

#include <optional>
#include <string>
#include <unordered_set>
#include <utility>

namespace Meshing
{
class MeshData3D;
struct RestrictedFacet;
} // namespace Meshing

namespace Meshing
{

/**
 * @brief CGAL Mesh_3's facet criteria, in its order, for the CGAL-style
 * refinement path (SurfaceDelaunayRefiner):
 *
 *  0. shape -- sin^2 of the smallest angle below sin^2(minAngleDegrees);
 *  1. distance -- the facet's weighted circumcenter further than
 *     chordDeviationTolerance from its surface Delaunay ball centre;
 *  2. same patch -- two surface-interior vertices on different surfaces
 *     (curve and corner vertices are ignored, as in CGAL's
 *     Facet_on_same_surface_criterion).
 *
 * Shape and distance are waived next to protecting balls (protectionExemption,
 * CGAL's Facet_criterion_visitor_with_features).
 */
class SurfaceFacetCriteria
{
public:
    /// The first failing criterion's index, then its quality; smaller is
    /// worse, so this orders CGAL's facet queue.
    using Badness = std::pair<int, double>;

    SurfaceFacetCriteria(const SurfaceMesh3DQualitySettings& settings, std::unordered_set<std::string> surfaceIds);

    /// nullopt when the facet satisfies every criterion.
    std::optional<Badness> findBadness(const FaceKey& face,
                                       const RestrictedFacet& facet,
                                       const MeshData3D& meshData) const;

private:
    double minimumAngleDegrees_;
    double distanceBound_;
    std::unordered_set<std::string> surfaceIds_;
};

} // namespace Meshing
