#pragma once

#include "Common/Types.h"
#include <map>
#include <string>
#include <vector>

namespace Meshing
{

/**
 * @brief Result of discretizing 2D geometry edges
 *
 * Contains the sampled points along edges and the mappings needed
 * for constrained Delaunay triangulation.
 */
struct DiscretizationResult2D
{
    /// Sampled points from corners and edge discretization
    std::vector<Point2D> points;

    /// Edge parameter t values for each point (empty vector for interior points)
    std::vector<std::vector<double>> tParameters;

    /// Geometry IDs for each point (edge IDs for boundary points)
    std::vector<std::vector<std::string>> geometryIds;

    /// Maps corner IDs to indices in points vector
    std::map<std::string, size_t> cornerIdToPointIndexMap;

    /// Maps edge IDs to ordered list of point indices along the edge
    std::map<std::string, std::vector<size_t>> edgeIdToPointIndicesMap;
};

} // namespace Meshing
