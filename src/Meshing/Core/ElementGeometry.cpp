#include "ElementGeometry.h"
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

namespace
{

template <size_t N>
bool gatherNodes(const MeshData& mesh,
                 const std::vector<size_t>& nodeIds,
                 std::array<const Node3D*, N>& nodes)
{
    if (nodeIds.size() != N)
    {
        return false;
    }

    for (size_t i = 0; i < N; ++i)
    {
        nodes[i] = mesh.getNode(nodeIds[i]);
        if (nodes[i] == nullptr)
        {
            return false;
        }
    }

    return true;
}

} // namespace

double computeVolume(const Point3DRef v0,
                     const Point3DRef v1,
                     const Point3DRef v2,
                     const Point3DRef v3)
{
    const auto edge1 = v1 - v0;
    const auto edge2 = v2 - v0;
    const auto edge3 = v3 - v0;

    const double scalarTriple = edge1.dot(edge2.cross(edge3));
    return std::abs(scalarTriple) / 6.0;
}

std::optional<double> computeVolume(const MeshData& mesh, const TetrahedralElement& element)
{
    std::array<const Node3D*, 4> nodes;
    if (!gatherNodes(mesh, element.getNodeIds(), nodes))
    {
        return std::nullopt;
    }

    return computeVolume(nodes[0]->getCoordinates(),
                         nodes[1]->getCoordinates(),
                         nodes[2]->getCoordinates(),
                         nodes[3]->getCoordinates());
}

double computeArea(const Point3DRef v0,
                   const Point3DRef v1,
                   const Point3DRef v2)
{
    const auto edge1 = v1 - v0;
    const auto edge2 = v2 - v0;
    return 0.5 * edge1.cross(edge2).norm();
}

std::optional<double> computeArea(const MeshData& mesh, const TriangleElement& element)
{
    std::array<const Node3D*, 3> nodes;
    if (!gatherNodes(mesh, element.getNodeIds(), nodes))
    {
        return std::nullopt;
    }

    return computeArea(nodes[0]->getCoordinates(),
                       nodes[1]->getCoordinates(),
                       nodes[2]->getCoordinates());
}

std::optional<CircumscribedSphere> computeCircumscribingSphere(const Point3DRef v0,
                                                               const Point3DRef v1,
                                                               const Point3DRef v2,
                                                               const Point3DRef v3)
{
    Eigen::Matrix3d A;
    A.row(0) = (v1 - v0).transpose();
    A.row(1) = (v2 - v0).transpose();
    A.row(2) = (v3 - v0).transpose();

    Eigen::Vector3d b;
    b(0) = 0.5 * (v1.squaredNorm() - v0.squaredNorm());
    b(1) = 0.5 * (v2.squaredNorm() - v0.squaredNorm());
    b(2) = 0.5 * (v3.squaredNorm() - v0.squaredNorm());

    const Eigen::FullPivLU<Eigen::Matrix3d> lu(A);
    if (!lu.isInvertible())
    {
        return std::nullopt;
    }

    const Point3D center = lu.solve(b);
    const double radius = (center - v0).norm();
    return CircumscribedSphere{center, radius};
}

std::optional<CircumscribedSphere> computeCircumscribingSphere(const MeshData& mesh, const TetrahedralElement& element)
{
    std::array<const Node3D*, 4> nodes;
    if (!gatherNodes(mesh, element.getNodeIds(), nodes))
    {
        return std::nullopt;
    }

    return computeCircumscribingSphere(nodes[0]->getCoordinates(),
                                       nodes[1]->getCoordinates(),
                                       nodes[2]->getCoordinates(),
                                       nodes[3]->getCoordinates());
}

std::optional<double> computeQuality(const MeshData& mesh, const TetrahedralElement& element)
{
    std::array<const Node3D*, 4> nodes;
    if (!gatherNodes(mesh, element.getNodeIds(), nodes))
    {
        return std::nullopt;
    }

    const double volume = computeVolume(nodes[0]->getCoordinates(),
                                        nodes[1]->getCoordinates(),
                                        nodes[2]->getCoordinates(),
                                        nodes[3]->getCoordinates());

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

std::optional<double> computeQuality(const MeshData& mesh, const TriangleElement& element)
{
    std::array<const Node3D*, 3> nodes;
    if (!gatherNodes(mesh, element.getNodeIds(), nodes))
    {
        return std::nullopt;
    }

    const double area = computeArea(nodes[0]->getCoordinates(),
                                    nodes[1]->getCoordinates(),
                                    nodes[2]->getCoordinates());

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

double computeShortestEdgeLength(const MeshData& mesh, const TetrahedralElement& element)
{
    std::array<const Node3D*, 4> nodes;
    if (!gatherNodes(mesh, element.getNodeIds(), nodes))
    {
        return 0.0;
    }

    double minLen = std::numeric_limits<double>::max();
    for (size_t i = 0; i < 4; ++i)
    {
        for (size_t j = i + 1; j < 4; ++j)
        {
            const double len = (nodes[i]->getCoordinates() - nodes[j]->getCoordinates()).norm();
            minLen = std::min(minLen, len);
        }
    }

    return minLen;
}

double computeCircumradiusToShortestEdgeRatio(const MeshData& mesh, const TetrahedralElement& element)
{
    constexpr double MIN_EDGE = 1e-15;

    const auto sphere = computeCircumscribingSphere(mesh, element);
    if (!sphere)
    {
        return 0.0;
    }

    const double shortestEdge = computeShortestEdgeLength(mesh, element);
    if (shortestEdge <= MIN_EDGE)
    {
        return std::numeric_limits<double>::infinity();
    }

    return sphere->radius / shortestEdge;
}

bool isSkinny(const MeshData& mesh, const TetrahedralElement& element, double threshold)
{
    const double ratio = computeCircumradiusToShortestEdgeRatio(mesh, element);
    if (ratio == 0.0)
    {
        return false;
    }

    return ratio > threshold;
}

} // namespace Meshing::ElementGeometry
