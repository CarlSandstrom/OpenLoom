#pragma once

#include "Meshing/Connectivity/FaceKey.h"
#include "Meshing/Core/3D/RCDT/NonManifoldEdgeRefiner.h"
#include "Meshing/Core/3D/RCDT/RCDTPointInserter.h"
#include "Meshing/Core/3D/RCDT/SurfaceProjector.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"

#include <unordered_set>

namespace Meshing
{

class MeshingContext3D;
class RestrictedTriangulation;
class RCDTTetQualityController;

/// Refines a restricted Delaunay triangulation in ambient 3D space.
///
/// Each refineStep() makes at most one insertion, for the first priority that
/// has work:
///
///   1. Split an encroached curve segment (RCDTPointInserter).
///   2. Insert a point for a bad restricted triangle (circumradius/edge or
///      chord deviation).
///   3. Insert the circumcenter of a skinny tetrahedron -- only when a
///      RCDTTetQualityController was supplied, i.e. when meshing a volume.
///   4. Repair a non-manifold edge of the restricted-face set
///      (NonManifoldEdgeRefiner).
///
/// Within a priority, candidates are taken in the order their containers yield
/// them, not worst first, so that order shapes the output mesh.
///
/// Priorities 2-4 demote to splitting a curve segment when their point would
/// encroach one (Shewchuk), and never insert within minimumEdgeLength_ of an
/// existing node. No priority inserts inside a protecting ball, which would
/// erode the crease protection it exists for (OPE-176). A candidate refused for
/// any of these reasons is recorded as unrefinable and never tried again.
///
/// Each priority's unrefinable set is never cleared: clearing them after every
/// segment split was measured on SaddleSurfaceMesh to rediscover the same
/// unfixable candidates about 100 times over, with no gain. Entries are keyed
/// by nodes or element ID, so a candidate an insertion restructures returns
/// under a new key; one whose key survives stays blocked even if its insertion
/// point has moved.
///
/// Slivers -- tetrahedra with an acceptable circumradius/edge ratio but poor
/// dihedral angles -- are not detected, and a near-flat tetrahedron whose
/// circumcenter lies far outside the mesh is left unrefined (see the
/// circumradius guard in refineBadTetrahedra()). Measured to affect a few
/// percent of tetrahedra.
class RCDTRefiner
{
public:
    RCDTRefiner(MeshingContext3D& context,
                RestrictedTriangulation& restrictedTriangulation,
                const SurfaceMesh3DQualitySettings& settings,
                double minimumEdgeLength,
                const RCDTTetQualityController* tetQualityController = nullptr);

    void refine();

private:
    MeshingContext3D* context_;
    RestrictedTriangulation* restrictedTriangulation_;
    SurfaceMesh3DQualitySettings settings_;
    const RCDTTetQualityController* tetQualityController_;
    SurfaceProjector surfaceProjector_;

    /// Candidates priorities 2 and 3 have given up on (see the class doc).
    std::unordered_set<FaceKey, FaceKeyHash> unrefinableTriangles_;
    std::unordered_set<size_t> unrefinableTetrahedra_;

    /// Size floor (see MinimumEdgeLengthEstimator). Bounds how short a segment,
    /// restricted triangle or non-manifold edge may get before it is left
    /// unrefined, how close an insertion may land to an existing node, and how
    /// large a tetrahedron's circumradius may be.
    double minimumEdgeLength_;

    RCDTPointInserter pointInserter_;
    NonManifoldEdgeRefiner nonManifoldEdgeRefiner_;

    /// Performs one refinement step. Returns true if any insertion was made.
    bool refineStep();

    /// Priority 3: inserts the circumcenter of the first refinable skinny
    /// tetrahedron, or splits the segment it would encroach. Returns false
    /// without doing anything when no tetQualityController_ was supplied.
    /// Returns true if an insertion or split was made.
    bool refineBadTetrahedra();

    /// Priority 3, then priority 4 if priority 3 found nothing to do.
    bool refineRemainingPriorities();
};

} // namespace Meshing
