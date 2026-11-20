#pragma once

#include "Common/Types.h"

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace Meshing
{

/// How an edge of a mesher's boundary departs from the coverage the CAD
/// topology calls for there.
enum class BoundaryDefectKind3D
{
    /// Fewer incident triangles than expected: a hole in the surface.
    MissingFace,
    /// More incident triangles than expected: the same piece of surface
    /// covered twice.
    ExcessFace,
    /// The expected number of triangles, but on the wrong surfaces.
    SurfaceMismatch
};

/// One boundary edge whose incident triangles do not match the CAD topology.
struct BoundaryDefect3D
{
    BoundaryDefectKind3D kind = BoundaryDefectKind3D::MissingFace;

    /// The surface the edge is short of a triangle on when one is missing,
    /// otherwise the surface carrying the excess.
    std::string surfaceId;

    std::array<Point3D, 2> endpoints;

    /// The edge's nodes in the numbering of the mesh the run produced; unset
    /// when the run produced no mesh.
    std::optional<std::array<NodeId, 2>> nodeIds;
};

/**
 * @brief The boundary defects a 3D mesher found in the mesh it built.
 *
 * Empty for a clean boundary. See ISurfaceMesher3D::getBoundaryDefects() and
 * IVolumeMesher3D::getBoundaryDefects().
 */
struct BoundaryDefectReport3D
{
    std::vector<BoundaryDefect3D> defects;
};

} // namespace Meshing
