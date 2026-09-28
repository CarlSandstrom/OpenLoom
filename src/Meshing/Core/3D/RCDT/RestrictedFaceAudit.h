#pragma once

#include "Meshing/Core/3D/RCDT/RestrictedFaceTypes.h"

#include <vector>

namespace Topology3D
{
class Topology3D;
} // namespace Topology3D

namespace Meshing
{
class MeshData3D;
} // namespace Meshing

namespace Meshing
{

/**
 * @brief Checks the restricted-face set against the coverage the CAD topology
 * calls for. RCDTMesher::meshVolume() refuses a boundary with a face missing.
 *
 * Free functions rather than a class: everything here is a pure function of
 * the restricted-face set plus the topology lookup, so there is no state worth
 * owning.
 *
 * Read-only and downstream of restriction: it reads the face set, it never
 * asks whether a face should have been in it, and it removes nothing -- the
 * old post-hoc chord and flap removal went with the RestrictedTriangulation
 * path (OPE-186), whose defects it repaired.
 */
namespace RestrictedFaceAudit
{

/// Every CAD curve against the surfaces it bounds, read from the topology: the
/// lookup findNonManifoldEdges() reads its expected counts from.
EdgeToAdjacentSurfacesMap buildEdgeToAdjacentSurfaces(const Topology3D::Topology3D& topology);

/// Every edge whose incident restricted faces do not match what the CAD
/// topology calls for there. A restricted set that satisfies the invariant
/// everywhere (what AmbientTetrahedronClassifier's flood fill requires, and
/// what a correct RCDT run should eventually produce) returns an empty vector.
///
/// The invariant is per-edge and depends on whether the edge lies ON a model
/// curve -- i.e. its two nodes are chain-adjacent along one, per meshData's
/// CurveSegmentManager:
///
///  * On a curve: exactly one incident face per surface adjacent to that
///    curve, as listed by Topology3D::Edge3D::getAdjacentSurfaceIds().
///    Two for an ordinary crease, one for a free boundary, and THREE OR
///    MORE at a junction where that many surfaces meet.
///  * Not on a curve (a surface interior, or a chord skipping a curve's
///    own sample points): exactly 2 incident faces, both restricted to
///    the same surface.
///
/// This is deliberately NOT the flat "every edge has exactly 2 faces" test it
/// replaces. That test states a closed-2-manifold requirement the models this
/// library targets do not all satisfy: in a conformal multi-material model --
/// a polycrystal or multiphase microstructure -- grain boundaries meet along
/// TRIPLE LINES where three boundary patches share one edge, and at quadruple
/// points where four triple lines meet. Three faces on such an edge is the
/// equilibrium configuration, not a defect, and Edge3D has always documented
/// its adjacency list as "usually 2, can be 1 (boundary) or >2
/// (non-manifold)". Reading the expected count off the topology rather than
/// assuming 2 is what lets a legitimate junction and an over-acceptance flap
/// be told apart.
///
/// The distinction also matters for the flap itself: the count test lumps
/// holes, duplicates and (in future) junctions into one number, so it cannot
/// serve as a quality gate, and any repair of the "keep the best 2, drop the
/// rest" shape built on it would silently destroy triple lines. See OPE-184.
std::vector<NonManifoldRestrictedEdge> findNonManifoldEdges(
    const RestrictedFaceMap& restrictedFaces,
    const EdgeToAdjacentSurfacesMap& edgeToAdjacentSurfaces,
    const MeshData3D& meshData);

} // namespace RestrictedFaceAudit

} // namespace Meshing
