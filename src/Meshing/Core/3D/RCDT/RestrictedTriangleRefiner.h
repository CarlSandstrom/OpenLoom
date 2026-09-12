#pragma once

#include "Meshing/Connectivity/FaceKey.h"

#include <unordered_set>

namespace Meshing
{

class MeshingContext3D;
class RCDTPointInserter;
class RestrictedTriangulation;

/// RCDTRefiner's priority 2: refines the restricted triangles that miss the
/// quality bounds -- circumradius/edge ratio or chord deviation, see
/// RestrictedTriangulation::getBadTriangles() -- by inserting a point on the
/// surface the triangle was classified to.
class RestrictedTriangleRefiner
{
public:
    RestrictedTriangleRefiner(const MeshingContext3D& context,
                              const RestrictedTriangulation& restrictedTriangulation,
                              double minimumEdgeLength);

    /// Inserts the point of the first refinable bad restricted triangle, or
    /// splits the segment it would encroach. Returns true if an insertion or
    /// split was made.
    bool refineNext(RCDTPointInserter& pointInserter);

private:
    const MeshingContext3D* context_;
    const RestrictedTriangulation* restrictedTriangulation_;
    double minimumEdgeLength_;

    /// Triangles given up on (see RCDTRefiner's class doc).
    std::unordered_set<FaceKey, FaceKeyHash> unrefinableTriangles_;
};

} // namespace Meshing
