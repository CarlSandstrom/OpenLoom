#include "Computer3D.h"

#include "ComputerGeneral.h"
#include "Meshing/Data/TriangleElement.h"
#include <Eigen/LU>
#include <cmath>
#include <limits>
#include <optional>

namespace Meshing
{

Computer3D::Computer3D(const MeshData3D& mesh) :
    mesh_(mesh)
{
}

double Computer3D::computeVolume(const TetrahedralElement& element) const
{
    auto [v0, v1, v2, v3] = getElementNodeCoordinates(element);
    const auto edge1 = v1 - v0;
    const auto edge2 = v2 - v0;
    const auto edge3 = v3 - v0;
    const double scalarTriple = edge1.dot(edge2.cross(edge3));
    return std::abs(scalarTriple) / 6.0;
}

double Computer3D::computeArea(const TriangleElement& element) const
{
    auto [v0, v1, v2] = getElementNodeCoordinates(element);
    const auto edge1 = v1 - v0;
    const auto edge2 = v2 - v0;
    return 0.5 * edge1.cross(edge2).norm();
}

std::optional<Computer3D::CircumscribedSphere> Computer3D::computeCircumscribingSphere(const TetrahedralElement& element) const
{
    auto [v0, v1, v2, v3] = getElementNodeCoordinates(element);

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

bool Computer3D::getIsPointInsideCircumscribingSphere(const TetrahedralElement& element, const Point3D& point, double tolerance) const
{
    if (const auto sphere = computeCircumscribingSphere(element))
    {
        return ComputerGeneral::getIsPointInsideCircumscribingSphere(*sphere, point, tolerance);
    }

    return false;
}

double Computer3D::getShortestEdgeLength(const TetrahedralElement& element) const
{
    auto nodeIds = element.getNodeIds();
    std::vector<const Node3D*> nodes;
    for (auto nodeId : nodeIds)
    {
        const auto* node = mesh_.getNode(nodeId);
        if (!node)
        {
            return 0.0;
        }
        nodes.push_back(node);
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

double Computer3D::getCircumradiusToShortestEdgeRatio(const TetrahedralElement& element) const
{
    constexpr double MIN_EDGE = 1e-15;

    const auto sphere = computeCircumscribingSphere(element);
    if (!sphere)
    {
        return 0.0;
    }

    const double shortestEdge = getShortestEdgeLength(element);
    if (shortestEdge <= MIN_EDGE)
    {
        return std::numeric_limits<double>::infinity();
    }

    return sphere->radius / shortestEdge;
}

bool Computer3D::isSkinny(const TetrahedralElement& element, double threshold) const
{
    const double ratio = getCircumradiusToShortestEdgeRatio(element);
    if (ratio == 0.0)
    {
        return false;
    }

    return ratio > threshold;
}

std::tuple<Point3D, Point3D, Point3D, Point3D> Computer3D::getElementNodeCoordinates(const TetrahedralElement& element) const
{
    auto nodeIds = element.getNodeIds();
    const auto* n0 = mesh_.getNode(nodeIds[0]);
    const auto* n1 = mesh_.getNode(nodeIds[1]);
    const auto* n2 = mesh_.getNode(nodeIds[2]);
    const auto* n3 = mesh_.getNode(nodeIds[3]);
    return {n0->getCoordinates(), n1->getCoordinates(), n2->getCoordinates(), n3->getCoordinates()};
}

std::tuple<Point3D, Point3D, Point3D> Computer3D::getElementNodeCoordinates(const TriangleElement& element) const
{
    auto nodeIds = element.getNodeIds();
    const auto* n0 = mesh_.getNode(nodeIds[0]);
    const auto* n1 = mesh_.getNode(nodeIds[1]);
    const auto* n2 = mesh_.getNode(nodeIds[2]);
    return {n0->getCoordinates(), n1->getCoordinates(), n2->getCoordinates()};
}

} // namespace Meshing
