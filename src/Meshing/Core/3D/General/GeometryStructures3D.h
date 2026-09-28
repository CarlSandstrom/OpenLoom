#pragma once

#include "Common/Types.h"
#include "Meshing/Data/ConstraintRole.h"
#include <string>

namespace Meshing
{

/**
 * @brief Represents a constrained triangular face in 3D mesh
 *
 * A subfacet is part of a constrained facet (surface) that must be
 * preserved in the final mesh. During refinement, facets may be
 * subdivided into multiple subfacets.
 */
struct ConstrainedSubfacet3D
{
    size_t nodeId1;                                  // First vertex node ID
    size_t nodeId2;                                  // Second vertex node ID
    size_t nodeId3;                                  // Third vertex node ID
    std::string geometryId;                          // Parent surface geometry ID
    ConstraintRole role = ConstraintRole::Boundary;  // Boundary or interior constraint
};

} // namespace Meshing