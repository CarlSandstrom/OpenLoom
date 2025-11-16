#include "ElementGeometry.h"
#include <Eigen/Core>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

#include <Eigen/LU>

namespace Meshing::ElementGeometry
{

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
    const auto& nodeIds = element.getNodeIds();
    if (nodeIds.size() != 4)
    {
        return std::nullopt;
    }

    std::array<const Node*, 4> nodes;
    for (size_t i = 0; i < 4; ++i)
    {
        nodes[i] = mesh.getNode(nodeIds[i]);
        if (nodes[i] == nullptr)
        {
            return std::nullopt;
        }
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
    const auto& nodeIds = element.getNodeIds();
    if (nodeIds.size() != 3)
    {
        return std::nullopt;
    }

    std::array<const Node*, 3> nodes;
    for (size_t i = 0; i < 3; ++i)
    {
        nodes[i] = mesh.getNode(nodeIds[i]);
        if (nodes[i] == nullptr)
        {
            return std::nullopt;
        }
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
    const auto& nodeIds = element.getNodeIds();
    if (nodeIds.size() != 4)
    {
        return std::nullopt;
    }

    std::array<const Node*, 4> nodes;
    for (size_t i = 0; i < 4; ++i)
    {
        nodes[i] = mesh.getNode(nodeIds[i]);
        if (nodes[i] == nullptr)
        {
            return std::nullopt;
        }
    }

    return computeCircumscribingSphere(nodes[0]->getCoordinates(),
                                       nodes[1]->getCoordinates(),
                                       nodes[2]->getCoordinates(),
                                       nodes[3]->getCoordinates());
}

std::optional<double> computeQuality(const MeshData& mesh, const TetrahedralElement& element)
{
    const auto& nodeIds = element.getNodeIds();
    if (nodeIds.size() != 4)
    {
        return std::nullopt;
    }

    std::array<const Node*, 4> nodes;
    for (size_t i = 0; i < 4; ++i)
    {
        nodes[i] = mesh.getNode(nodeIds[i]);
        if (nodes[i] == nullptr)
        {
            return std::nullopt;
        }
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
    const auto& nodeIds = element.getNodeIds();
    if (nodeIds.size() != 3)
    {
        return std::nullopt;
    }

    std::array<const Node*, 3> nodes;
    for (size_t i = 0; i < 3; ++i)
    {
        nodes[i] = mesh.getNode(nodeIds[i]);
        if (nodes[i] == nullptr)
        {
            return std::nullopt;
        }
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

} // namespace Meshing::ElementGeometry
