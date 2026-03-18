#pragma once

#include "Common/Types.h"
#include <algorithm>
#include <array>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace Meshing
{

/**
 * @brief Represents a constrained segment from CAD topology
 */
struct ConstrainedSegment
{
    size_t startNodeId;
    size_t endNodeId;
    std::string topologyEdgeId;

    ConstrainedSegment(size_t start, size_t end, const std::string& edgeId) :
        startNodeId(start), endNodeId(end), topologyEdgeId(edgeId) {}
};

/**
 * @brief Represents a triangular subfacet from CAD surface
 *
 * Each CAD surface is triangulated into multiple subfacets.
 * These subfacets must appear in the final mesh.
 */
struct ConstrainedSubfacet
{
    std::array<size_t, 3> nodeIds; // Triangle vertices
    std::string topologySurfaceId; // Parent surface ID

    ConstrainedSubfacet(size_t n0, size_t n1, size_t n2, const std::string& surfaceId) :
        nodeIds({n0, n1, n2}), topologySurfaceId(surfaceId) {}

    ConstrainedSubfacet(const std::array<size_t, 3>& nodes, const std::string& surfaceId) :
        nodeIds(nodes), topologySurfaceId(surfaceId) {}
};

} // namespace Meshing
