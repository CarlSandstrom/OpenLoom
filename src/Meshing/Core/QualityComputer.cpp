#include "QualityComputer.h"
#include <Eigen/Core>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include <Eigen/LU>

namespace Meshing::ElementGeometry
{

std::optional<double> computeQuality(const MeshData3D& mesh, const TetrahedralElement& element)
{
    std::array<const Node3D*, 4> nodes;
    if (!gatherNodes(mesh, element.getNodeIds(), nodes))
    {
        return std::nullopt;
    }

    const auto& v0 = nodes[0]->getCoordinates();
    const auto& v1 = nodes[1]->getCoordinates();
    const auto& v2 = nodes[2]->getCoordinates();
    const auto& v3 = nodes[3]->getCoordinates();

    const auto edge1 = v1 - v0;
    const auto edge2 = v2 - v0;
    const auto edge3 = v3 - v0;
    const double volume = std::abs(edge1.dot(edge2.cross(edge3))) / 6.0;

    constexpr std::array<std::pair<size_t, size_t>, 6> edges = {{{0, 1},
                                                                 {0, 2},
                                                                 {0, 3},
                                                                 {1, 2},
                                                                 {1, 3},
                                                                 {2, 3}}};

    double sumSquaredLengths = 0.0;
    for (const auto& edge : edges)
    {
        const auto length = (nodes[edge.first]->getCoordinates() - nodes[edge.second]->getCoordinates()).norm();
        sumSquaredLengths += length * length;
    }

    if (sumSquaredLengths <= std::numeric_limits<double>::epsilon())
    {
        return std::nullopt;
    }

    const double numerator = 72.0 * std::sqrt(3.0) * volume;
    const double denominator = std::pow(sumSquaredLengths, 1.5);
    if (denominator <= std::numeric_limits<double>::epsilon())
    {
        return std::nullopt;
    }

    return numerator / denominator;
}

std::optional<double> computeQuality(const MeshData3D& mesh, const TriangleElement& element)
{
    std::array<const Node3D*, 3> nodes;
    if (!gatherNodes(mesh, element.getNodeIds(), nodes))
    {
        return std::nullopt;
    }

    const auto& v0 = nodes[0]->getCoordinates();
    const auto& v1 = nodes[1]->getCoordinates();
    const auto& v2 = nodes[2]->getCoordinates();
    const double area = 0.5 * (v1 - v0).cross(v2 - v0).norm();

    constexpr std::array<std::pair<size_t, size_t>, 3> edges = {{{0, 1},
                                                                 {1, 2},
                                                                 {2, 0}}};

    double sumSquaredLengths = 0.0;
    for (const auto& edge : edges)
    {
        const auto length = (nodes[edge.first]->getCoordinates() - nodes[edge.second]->getCoordinates()).norm();
        sumSquaredLengths += length * length;
    }

    if (sumSquaredLengths <= std::numeric_limits<double>::epsilon())
    {
        return std::nullopt;
    }

    const double numerator = 4.0 * std::sqrt(3.0) * area;
    return numerator / sumSquaredLengths;
}

} // namespace Meshing::ElementGeometry
