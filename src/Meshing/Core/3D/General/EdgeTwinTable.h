#pragma once

#include <map>
#include <string>
#include <vector>

namespace Meshing
{

/// Orientation of a surface's traversal of a shared edge relative to the edge's canonical
/// direction (start corner → end corner as stored in Edge3D).
///
/// Same:     the surface traverses the edge from start corner to end corner.
/// Reversed: the surface traverses the edge from end corner to start corner.
///
/// For a consistently oriented manifold solid, any two surfaces sharing an edge
/// traverse it in opposite directions. The first surface listed in Edge3D is
/// assigned Same (canonical); all others are assigned Reversed.
enum class TwinOrientation
{
    Same,
    Reversed
};

/// One surface's membership in a shared topology edge group.
struct EdgeTwinEntry
{
    std::string surfaceId;
    TwinOrientation orientation;
};

/// Maps each shared topology edge to the surfaces that share it, with orientations.
///
/// Only edges with two or more adjacent surfaces are present.
/// Boundary edges (single adjacent surface) are omitted.
using EdgeTwinTable = std::map<std::string, std::vector<EdgeTwinEntry>>;

} // namespace Meshing
