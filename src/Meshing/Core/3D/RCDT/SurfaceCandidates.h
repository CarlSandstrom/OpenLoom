#pragma once

#include "Meshing/Core/3D/RCDT/RestrictedFaceTypes.h"

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace Topology3D
{
class Topology3D;
} // namespace Topology3D

namespace Meshing
{

/**
 * @brief Resolves the CAD surfaces a mesh node can lie on, from the geometry
 * ids it carries.
 *
 * A node's geometry ids name whichever CAD entity it was sampled from, which
 * is not always a surface: a node on a crease carries a curve id, one at a
 * junction carries a corner id. Restriction is decided per surface, so those
 * have to be resolved to the surfaces they belong to before any face can be
 * classified -- a curve to the surfaces it bounds, a corner to the surfaces
 * meeting there.
 *
 * Read out of the topology once and then constant, so this answers from its
 * own tables rather than walking Topology3D per query.
 */
class SurfaceCandidates
{
public:
    SurfaceCandidates() = default;

    /// Reads every surface, curve-to-surface and corner-to-surface
    /// relationship the resolution needs out of the topology.
    explicit SurfaceCandidates(const Topology3D::Topology3D& topology);

    /// The CAD surfaces a node tagged with these geometry ids can lie on. A
    /// surface id resolves to itself, a curve id to the surfaces it bounds, a
    /// corner id to the surfaces meeting there; an id naming none of those
    /// contributes nothing.
    ///
    /// Intersecting this across a face's three nodes is what narrows a face to
    /// its candidate surfaces -- a face can only lie on a surface all three of
    /// its corners lie on.
    std::unordered_set<std::string> effectiveSurfaceIds(const std::vector<std::string>& geometryIds) const;

    /// Every surface in the topology.
    const std::unordered_set<std::string>& getSurfaceIds() const { return surfaceIds_; }

    /// CAD curve id -> the surfaces that curve bounds. Exposed because
    /// RestrictedFaceAudit reads the same relationship to work out how many
    /// faces an edge should carry.
    const EdgeToAdjacentSurfacesMap& getEdgeToAdjacentSurfaces() const { return edgeToAdjacentSurfaces_; }

private:
    std::unordered_set<std::string> surfaceIds_;
    EdgeToAdjacentSurfacesMap edgeToAdjacentSurfaces_;
    std::unordered_map<std::string, std::vector<std::string>> cornerToAdjacentSurfaces_;
};

} // namespace Meshing
