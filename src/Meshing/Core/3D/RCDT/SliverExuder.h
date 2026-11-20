#pragma once

#include "Common/Types.h"
#include "Meshing/Connectivity/FaceKey.h"
#include "Meshing/Core/3D/RCDT/RestrictedFaceTypes.h"

#include <array>
#include <unordered_set>
#include <utility>
#include <vector>

namespace Meshing
{
class MeshingContext3D;
class WeightedDualRestriction;
} // namespace Meshing

namespace Meshing
{

/**
 * @brief Sliver exudation as CGAL Mesh_3 does it (Slivers_exuder): raises a
 * vertex's weight until the slivers around it leave the regular
 * triangulation, keeping the weight that makes the worst tetrahedron around
 * the vertex best. Runs after TetrahedronDelaunayRefiner, on the weighted
 * tetrahedralization, before the ambient tetrahedra are removed.
 *
 *  - A sliver is a tetrahedron inside the domain whose smallest dihedral
 *    angle is below the bound. Slivers are visited worst first, each of their
 *    vertices in turn until one change improves them.
 *  - A weight rises at most to SQUARED_DELTA times the squared distance to
 *    the vertex's nearest neighbour (CGAL's sq_delta), and a weight that
 *    would hide another vertex is refused. Only unweighted vertices are
 *    pumped: corner and curve vertices carry protecting balls.
 *  - Positions never move, and a change is undone unless every face it
 *    touches keeps its restriction: the restricted surface is exactly what
 *    refinement produced, so the boundary mesh does not change.
 *
 * No perturbation (CGAL's perturb_mesh_3): that moves vertices, which needs
 * vertex removal from the regular triangulation.
 */
class SliverExuder
{
public:
    SliverExuder(MeshingContext3D& context,
                 const WeightedDualRestriction& restriction,
                 const RestrictedFaceMap& restrictedFaces,
                 double sliverAngleDegrees);

    void exude();

private:
    struct Candidate
    {
        double weight;
        double worstAngle;
    };

    MeshingContext3D* context_;
    const WeightedDualRestriction* restriction_;
    const RestrictedFaceMap* restrictedFaces_;
    double sliverAngleDegrees_;

    std::vector<std::pair<double, ElementId>> findSlivers() const;
    bool pumpVertex(NodeId nodeId);
    std::vector<Candidate> findCandidates(NodeId nodeId) const;
    bool apply(NodeId nodeId, const Candidate& candidate);
    bool keepsRestriction(NodeId nodeId, const std::unordered_set<FaceKey, FaceKeyHash>& oldFaces) const;
    void restore(NodeId nodeId, double weight, const std::vector<std::array<NodeId, 4>>& removed);
};

} // namespace Meshing
