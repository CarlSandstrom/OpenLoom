#include "SurfaceMeshTopology.h"

#include "Meshing/Data/3D/SurfaceMesh3D.h"

#include <algorithm>
#include <sstream>
#include <utility>

namespace TestSupport
{

namespace
{

constexpr size_t MAXIMUM_EDGES_DESCRIBED = 5;

} // namespace

long long SurfaceMeshTopology::eulerCharacteristic() const
{
    return static_cast<long long>(vertices.size()) - static_cast<long long>(trianglesPerEdge.size()) +
           static_cast<long long>(triangleCount);
}

std::vector<SurfaceMeshEdge> SurfaceMeshTopology::edgesNotSharedBy(size_t expectedCount) const
{
    std::vector<SurfaceMeshEdge> offending;
    for (const auto& [edge, count] : trianglesPerEdge)
    {
        if (count != expectedCount)
            offending.push_back(edge);
    }
    return offending;
}

std::string SurfaceMeshTopology::describe(const std::vector<SurfaceMeshEdge>& edges) const
{
    std::ostringstream description;
    const size_t shown = std::min(edges.size(), MAXIMUM_EDGES_DESCRIBED);
    for (size_t i = 0; i < shown; ++i)
    {
        description << "\n  edge (" << edges[i].first << ", " << edges[i].second
                    << ") is on " << trianglesPerEdge.at(edges[i]) << " triangle(s)";
    }
    if (edges.size() > shown)
        description << "\n  ... and " << (edges.size() - shown) << " more";
    return description.str();
}

SurfaceMeshTopology computeSurfaceMeshTopology(const Meshing::SurfaceMesh3D& surfaceMesh)
{
    SurfaceMeshTopology topology;
    topology.triangleCount = surfaceMesh.triangles.size();

    for (const auto& triangle : surfaceMesh.triangles)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            topology.vertices.insert(triangle[i]);

            size_t first = triangle[i];
            size_t second = triangle[(i + 1) % 3];
            if (first > second)
                std::swap(first, second);
            ++topology.trianglesPerEdge[{first, second}];
        }
    }
    return topology;
}

} // namespace TestSupport
