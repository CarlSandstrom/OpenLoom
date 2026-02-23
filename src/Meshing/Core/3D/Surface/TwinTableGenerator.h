#pragma once

#include "Meshing/Core/3D/General/EdgeTwinTable.h"

namespace Topology3D
{
class Topology3D;
}

namespace Meshing
{

/// Generates the EdgeTwinTable from topology alone.
///
/// Reads edge adjacency directly from Edge3D::getAdjacentSurfaceIds().
/// No geometry is involved — this step is purely topological.
///
/// Orientation convention:
///   The first adjacent surface listed in the Edge3D is assigned Same (canonical direction).
///   All subsequent adjacent surfaces are assigned Reversed.
///
/// Usage:
/// @code
///   EdgeTwinTable table = TwinTableGenerator::generate(topology);
/// @endcode
class TwinTableGenerator
{
public:
    static EdgeTwinTable generate(const Topology3D::Topology3D& topology);
};

} // namespace Meshing
