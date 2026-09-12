#pragma once

#include <cstddef>
#include <unordered_set>

namespace Meshing
{

class MeshingContext3D;
class RCDTPointInserter;
class RCDTTetQualityController;
class RestrictedTriangulation;

/// RCDTRefiner's priority 3, used only when meshing a volume: inserts the
/// circumcenters of skinny tetrahedra, those whose circumradius-to-shortest-
/// edge ratio exceeds the bound.
///
/// Slivers -- tetrahedra with an acceptable circumradius/edge ratio but poor
/// dihedral angles -- are not detected, and a near-flat tetrahedron whose
/// circumcenter lies far outside the mesh is left unrefined (see the
/// circumradius guard in refineNext()). Measured to affect a few percent of
/// tetrahedra.
class TetrahedronQualityRefiner
{
public:
    TetrahedronQualityRefiner(MeshingContext3D& context,
                              const RestrictedTriangulation& restrictedTriangulation,
                              const RCDTTetQualityController& tetrahedronQualityController,
                              double circumradiusToShortestEdgeRatio,
                              double minimumEdgeLength);

    /// Inserts the circumcenter of the first refinable skinny tetrahedron, or
    /// splits the segment it would encroach. Returns true if an insertion or
    /// split was made.
    bool refineNext(RCDTPointInserter& pointInserter);

private:
    MeshingContext3D* context_;
    const RestrictedTriangulation* restrictedTriangulation_;
    const RCDTTetQualityController* tetrahedronQualityController_;
    double circumradiusToShortestEdgeRatio_;
    double minimumEdgeLength_;

    /// Tetrahedra given up on (see RCDTRefiner's class doc).
    std::unordered_set<size_t> unrefinableTetrahedra_;
};

} // namespace Meshing
