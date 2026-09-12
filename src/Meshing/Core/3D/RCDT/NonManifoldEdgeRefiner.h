#pragma once

#include "Meshing/Connectivity/EdgeKey.h"

#include <unordered_set>

namespace Meshing
{

class MeshingContext3D;
class RCDTPointInserter;
class RestrictedTriangulation;

/// RCDTRefiner's priority 4: repairs non-manifold edges of the restricted-face
/// set (see RestrictedTriangulation::findNonManifoldEdges()) by inserting a
/// point at the defect. If the edge's endpoints are joined by a curve segment,
/// that segment is split, which keeps the new point exactly on the crease;
/// projecting onto one of the surfaces instead was measured to move such a
/// defect, not resolve it.
class NonManifoldEdgeRefiner
{
public:
    NonManifoldEdgeRefiner(const MeshingContext3D& context,
                           const RestrictedTriangulation& restrictedTriangulation,
                           double minimumEdgeLength);

    /// Repairs the first refinable non-manifold restricted-face edge by
    /// splitting the curve segment joining its endpoints if there is one,
    /// otherwise by inserting its midpoint projected onto the defect's surface.
    /// Returns true if an insertion or split was made.
    bool refineNext(RCDTPointInserter& pointInserter);

private:
    const MeshingContext3D* context_;
    const RestrictedTriangulation* restrictedTriangulation_;
    double minimumEdgeLength_;

    /// Defects given up on (see RCDTRefiner's class doc).
    std::unordered_set<EdgeKey, EdgeKeyHash> unrefinableNonManifoldEdges_;
};

} // namespace Meshing
