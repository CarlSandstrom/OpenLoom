#include "Computer.h"

#include "Meshing/Data/TriangleElement.h"
#include <cmath>
#include <limits>
#include <optional>

namespace Meshing
{

Computer::Computer(const MeshData3D& mesh) :
    mesh_(mesh)
{
}

double Computer::computeVolume(const TetrahedralElement& element)
{
    auto [v0, v1, v2, v3] = getElementNodeCoordinates(element);
    const auto edge1 = v1 - v0;
    const auto edge2 = v2 - v0;
    const auto edge3 = v3 - v0;
    const double scalarTriple = edge1.dot(edge2.cross(edge3));
    return std::abs(scalarTriple) / 6.0;
}

double Computer::computeArea(const TriangleElement& element) const
{
    auto [v0, v1, v2] = getElementNodeCoordinates(element);
    const auto edge1 = v1 - v0;
    const auto edge2 = v2 - v0;
    return 0.5 * edge1.cross(edge2).norm();
}

std::optional<Computer::CircumscribedSphere> Computer::computeCircumscribingSphere(const TetrahedralElement& element) const
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

bool Computer::getIsPointInsideCircumscribingSphere(const CircumscribedSphere& sphere,
                                                    const Point3DRef point,
                                                    double tolerance)
{
    const double radiusSq = sphere.radius * sphere.radius;
    const double distanceSq = (point - sphere.center).squaredNorm();
    return distanceSq <= radiusSq + tolerance;
}

bool Computer::getIsPointInsideCircumscribingSphere(const TetrahedralElement& element, const Point3D& point) const
{
    if (const auto sphere = computeCircumscribingSphere(element))
    {
        return getIsPointInsideCircumscribingSphere(*sphere, point);
    }

    return false;
}

double Computer::getShortestEdgeLength(const TetrahedralElement& element) const
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

double Computer::getCircumradiusToShortestEdgeRatio(const TetrahedralElement& element) const
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

bool Computer::isSkinny(const TetrahedralElement& element, double threshold) const
{
    const double ratio = getCircumradiusToShortestEdgeRatio(element);
    if (ratio == 0.0)
    {
        return false;
    }

    return ratio > threshold;
}

std::tuple<Point3D, Point3D, Point3D, Point3D> Computer::getElementNodeCoordinates(const TetrahedralElement& element) const
{
    auto nodeIds = element.getNodeIds();
    const auto* n0 = mesh_.getNode(nodeIds[0]);
    const auto* n1 = mesh_.getNode(nodeIds[1]);
    const auto* n2 = mesh_.getNode(nodeIds[2]);
    const auto* n3 = mesh_.getNode(nodeIds[3]);
    return {n0->getCoordinates(), n1->getCoordinates(), n2->getCoordinates(), n3->getCoordinates()};
}

std::tuple<Point3D, Point3D, Point3D> Computer::getElementNodeCoordinates(const TriangleElement& element) const
{
    auto nodeIds = element.getNodeIds();
    const auto* n0 = mesh_.getNode(nodeIds[0]);
    const auto* n1 = mesh_.getNode(nodeIds[1]);
    const auto* n2 = mesh_.getNode(nodeIds[2]);
    return {n0->getCoordinates(), n1->getCoordinates(), n2->getCoordinates()};
}

std::optional<Computer::CircumCircle2D> Computer::computeCircumcircle(const TriangleElement& tri,
                                                                      const std::unordered_map<size_t, Point2D>& nodeCoords)
{
    const auto& nodes = tri.getNodeIdArray();

    auto it0 = nodeCoords.find(nodes[0]);
    auto it1 = nodeCoords.find(nodes[1]);
    auto it2 = nodeCoords.find(nodes[2]);

    if (it0 == nodeCoords.end() || it1 == nodeCoords.end() || it2 == nodeCoords.end())
    {
        return std::nullopt;
    }

    const Point2D& p0 = it0->second;
    const Point2D& p1 = it1->second;
    const Point2D& p2 = it2->second;

    const double ax = p1.x() - p0.x();
    const double ay = p1.y() - p0.y();
    const double bx = p2.x() - p0.x();
    const double by = p2.y() - p0.y();

    const double d = 2.0 * (ax * by - ay * bx);

    if (std::abs(d) < 1e-10)
    {
        // Degenerate triangle
        return std::nullopt;
    }

    const double aSq = ax * ax + ay * ay;
    const double bSq = bx * bx + by * by;

    const double cx = (by * aSq - ay * bSq) / d;
    const double cy = (ax * bSq - bx * aSq) / d;

    CircumCircle2D circle;
    circle.center = Point2D(p0.x() + cx, p0.y() + cy);
    circle.radiusSquared = cx * cx + cy * cy;

    return circle;
}

bool Computer::isPointInsideCircumcircle(const CircumCircle2D& circle, const Point2D& point)
{
    const double dx = point.x() - circle.center.x();
    const double dy = point.y() - circle.center.y();
    const double distSquared = dx * dx + dy * dy;

    return distSquared < circle.radiusSquared - 1e-10;
}

} // namespace Meshing
