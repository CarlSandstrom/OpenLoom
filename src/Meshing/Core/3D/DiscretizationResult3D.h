#pragma once

#include "Common/Types.h"
#include <map>
#include <string>
#include <vector>

namespace Meshing
{

/**
 * @brief Result of discretizing 3D geometry boundaries
 *
 * Contains the sampled points along corners, edges, and surfaces,
 * along with the mappings needed for constraint setup and Delaunay triangulation.
 */
struct DiscretizationResult3D
{
    /// Sampled points from corners, edge interiors, and surface interiors
    std::vector<Point3D> points;

    /// Edge parameter t values for each point (empty vector for non-edge points)
    std::vector<std::vector<double>> edgeParameters;

    /// Geometry IDs for each point (corner/edge/surface IDs)
    std::vector<std::vector<std::string>> geometryIds;

    /// Maps corner IDs to indices in points vector
    std::map<std::string, size_t> cornerIdToPointIndexMap;

    /// Maps edge IDs to ordered list of point indices along the edge (including endpoints)
    std::map<std::string, std::vector<size_t>> edgeIdToPointIndicesMap;

    /// Maps surface IDs to list of interior point indices on the surface
    std::map<std::string, std::vector<size_t>> surfaceIdToPointIndicesMap;
};

} // namespace Meshing
