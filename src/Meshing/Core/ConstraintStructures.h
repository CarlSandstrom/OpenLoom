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

struct PairHash
{
    size_t operator()(const std::pair<size_t, size_t>& p) const
    {
        return std::hash<size_t>()(p.first) ^ (std::hash<size_t>()(p.second) << 1);
    }
};

struct TriangleHash
{
    size_t operator()(const std::array<size_t, 3>& t) const
    {
        std::array<size_t, 3> sorted = t;
        std::sort(sorted.begin(), sorted.end());
        return std::hash<size_t>()(sorted[0]) ^
               (std::hash<size_t>()(sorted[1]) << 1) ^
               (std::hash<size_t>()(sorted[2]) << 2);
    }
};

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

/**
 * @brief Represents a constrained facet (CAD surface with its triangulation)
 */
struct ConstrainedFacet
{
    std::string topologySurfaceId;
    std::vector<size_t> boundaryNodeIds;        // Ordered boundary loop
    std::vector<ConstrainedSubfacet> subfacets; // Triangulation

    ConstrainedFacet(const std::string& surfaceId) :
        topologySurfaceId(surfaceId) {}

    void addSubfacet(size_t n0, size_t n1, size_t n2)
    {
        subfacets.emplace_back(n0, n1, n2, topologySurfaceId);
    }

    size_t getSubfacetCount() const { return subfacets.size(); }
};

/**
 * @brief Collection of constraints from CAD geometry
 */
struct ConstraintSet
{
    std::vector<ConstrainedSegment> segments;
    std::vector<ConstrainedFacet> facets;

    void clear()
    {
        segments.clear();
        facets.clear();
    }

    size_t getSegmentCount() const { return segments.size(); }
    size_t getFacetCount() const { return facets.size(); }

    size_t getTotalSubfacetCount() const
    {
        size_t count = 0;
        for (const auto& facet : facets)
        {
            count += facet.getSubfacetCount();
        }
        return count;
    }
};

} // namespace Meshing
