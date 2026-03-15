#pragma once

#include <map>
#include <string>
#include <vector>

namespace Meshing
{

/// One surface's membership in a shared topology edge group.
struct EdgeTwinEntry
{
    std::string surfaceId;
};

/// Maps each shared topology edge to the surfaces that share it.
///
/// Only edges with two or more adjacent surfaces are present.
/// Boundary edges (single adjacent surface) are omitted.
using EdgeTwinTable = std::map<std::string, std::vector<EdgeTwinEntry>>;

} // namespace Meshing
