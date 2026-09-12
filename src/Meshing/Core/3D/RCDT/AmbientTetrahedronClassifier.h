#pragma once

#include <cstddef>
#include <unordered_set>

namespace Meshing
{
class MeshData3D;
class RestrictedTriangulation;
} // namespace Meshing

namespace Meshing
{

/**
 * @brief Classifies every tetrahedron of the ambient RCDT tetrahedralization
 * as "solid" (part of the object being meshed) or "ambient".
 *
 * RCDT deliberately keeps the full space triangulated throughout refinement
 * -- see RCDTMesher's class docs -- so that every restricted (boundary) face
 * always has tetrahedra on both sides for the dual Voronoi edge computation.
 * "Ambient" is everything on the wrong side of the object's boundary: the
 * true exterior beyond the bounding supertet, and any interior voids/holes.
 * The ambient tetrahedralization does not distinguish holes from the true
 * exterior -- a through-hole connects the two without crossing any
 * restricted face -- and neither does this classifier: a single flood fill
 * from the bounding-supertet tetrahedra, crossing only non-restricted faces,
 * reaches both at once.
 *
 * Recomputed from scratch on every classify() call rather than maintained
 * incrementally -- see OPE-168: the surrounding refinement loop already pays
 * O(tet count) per iteration in two other places, the MeshConnectivity rebuild
 * (RestrictedTriangleRefiner and RCDTPointInserter each construct one per
 * step, as does classify() itself) and MeshQueries3D::findSkinnyTetrahedra(),
 * which rescans every element in the very function that calls classify(). So a
 * full recompute here doesn't change the overall complexity class -- and it's
 * a much simpler, easier to verify reference to optimize against later.
 * (Checked OPE-201: this argument used to cite
 * RestrictedTriangulation::getBadTriangles() as a third such place. That one
 * has since become incrementally maintained and now costs only O(bad
 * restricted face count), so it is no longer evidence for this.)
 */
class AmbientTetrahedronClassifier
{
public:
    /// Returns the ids of every ambient tetrahedron (true exterior and hole
    /// interiors alike); test membership with contains().
    static std::unordered_set<size_t> classify(const MeshData3D& meshData,
                                               const RestrictedTriangulation& restrictedTriangulation);
};

} // namespace Meshing
