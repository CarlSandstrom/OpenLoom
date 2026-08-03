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
 * O(tet count) per iteration in several other places (MeshConnectivity
 * rebuild, RestrictedTriangulation::getBadTriangles(),
 * MeshQueries3D::findSkinnyTetrahedra()), so a full recompute here doesn't
 * change the overall complexity class -- and it's a much simpler, easier to
 * verify reference to optimize against later.
 */
class AmbientTetrahedronClassifier
{
public:
    void classify(const MeshData3D& meshData, const RestrictedTriangulation& restrictedTriangulation);

    bool isAmbient(size_t tetId) const;

private:
    std::unordered_set<size_t> ambientTetIds_;
};

} // namespace Meshing
