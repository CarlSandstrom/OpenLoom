#include "Meshing/Core/3D/General/ElementGeometry3D.h"
#include "Meshing/Core/3D/General/GeometryUtilities3D.h"
#include "Meshing/Data/2D/TriangleElement.h"

#include <Eigen/LU>
#include <array>
#include <cmath>

namespace Meshing
{

ElementGeometry3D::ElementGeometry3D(const MeshData3D& mesh) :
    mesh_(mesh)
{
}

double ElementGeometry3D::computeVolume(const TetrahedralElement& element) const
{
    auto [v0, v1, v2, v3] = getElementNodeCoordinates(element);
    const auto edge1 = v1 - v0;
    const auto edge2 = v2 - v0;
    const auto edge3 = v3 - v0;
    const double scalarTriple = edge1.dot(edge2.cross(edge3));
    return std::abs(scalarTriple) / 6.0;
}

double ElementGeometry3D::computeArea(const TriangleElement& element) const
{
    auto [v0, v1, v2] = getElementNodeCoordinates(element);
    const auto edge1 = v1 - v0;
    const auto edge2 = v2 - v0;
    return 0.5 * edge1.cross(edge2).norm();
}

Point3D ElementGeometry3D::computeNormal(const TriangleElement& element) const
{
    auto [v0, v1, v2] = getElementNodeCoordinates(element);
    const auto cross = (v1 - v0).cross(v2 - v0);
    const double norm = cross.norm();
    if (norm < 1e-14)
    {
        return Point3D::Zero();
    }
    return cross / norm;
}

std::optional<EquatorialSphere> ElementGeometry3D::computeCircumcircle(const TriangleElement& element) const
{
    auto [v0, v1, v2] = getElementNodeCoordinates(element);
    if ((v1 - v0).cross(v2 - v0).squaredNorm() < 1e-24)
    {
        return std::nullopt;
    }
    return GeometryUtilities3D::createEquatorialSphere(v0, v1, v2);
}

std::optional<CircumscribedSphere> ElementGeometry3D::computeCircumscribingSphere(const TetrahedralElement& element) const
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

Point3D ElementGeometry3D::computeCentroid(const TetrahedralElement& element) const
{
    auto [v0, v1, v2, v3] = getElementNodeCoordinates(element);
    return Point3D((v0.x() + v1.x() + v2.x() + v3.x()) / 4.0,
                   (v0.y() + v1.y() + v2.y() + v3.y()) / 4.0,
                   (v0.z() + v1.z() + v2.z() + v3.z()) / 4.0);
}

std::optional<Point3D> ElementGeometry3D::computeOrthocenter(const TetrahedralElement& element) const
{
    const auto& nodeIds = element.getNodeIds();
    std::array<const Node3D*, 4> nodes;
    for (size_t i = 0; i < 4; ++i)
        nodes[i] = mesh_.getNode(nodeIds[i]);

    const Point3D& v0 = nodes[0]->getCoordinates();
    const double w0 = nodes[0]->getWeight();

    Eigen::Matrix3d A;
    Eigen::Vector3d b;
    for (size_t i = 1; i < 4; ++i)
    {
        const Point3D& vi = nodes[i]->getCoordinates();
        A.row(i - 1) = (vi - v0).transpose();
        b(i - 1) = 0.5 * (vi.squaredNorm() - v0.squaredNorm() - (nodes[i]->getWeight() - w0));
    }

    const Eigen::FullPivLU<Eigen::Matrix3d> lu(A);
    if (!lu.isInvertible())
    {
        return std::nullopt;
    }
    return Point3D(lu.solve(b));
}

std::optional<Point3D> ElementGeometry3D::computeOrthocenter(const TriangleElement& element) const
{
    const auto& nodeIds = element.getNodeIds();
    const Node3D* n0 = mesh_.getNode(nodeIds[0]);
    const Node3D* n1 = mesh_.getNode(nodeIds[1]);
    const Node3D* n2 = mesh_.getNode(nodeIds[2]);

    // c = v0 + a*u + b*v, with equal power distance to all three vertices:
    // 2 (c - v0) . u = |u|^2 + w0 - w1, and likewise for v.
    const Point3D& v0 = n0->getCoordinates();
    const Point3D u = n1->getCoordinates() - v0;
    const Point3D v = n2->getCoordinates() - v0;

    Eigen::Matrix2d A;
    A << u.dot(u), u.dot(v), u.dot(v), v.dot(v);
    const Eigen::Vector2d b(0.5 * (u.dot(u) + n0->getWeight() - n1->getWeight()),
                            0.5 * (v.dot(v) + n0->getWeight() - n2->getWeight()));

    const Eigen::FullPivLU<Eigen::Matrix2d> lu(A);
    if (!lu.isInvertible())
    {
        return std::nullopt;
    }
    const Eigen::Vector2d coefficients = lu.solve(b);
    return Point3D(v0 + coefficients(0) * u + coefficients(1) * v);
}

std::tuple<Point3D, Point3D, Point3D, Point3D> ElementGeometry3D::getElementNodeCoordinates(const TetrahedralElement& element) const
{
    auto nodeIds = element.getNodeIds();
    const auto* n0 = mesh_.getNode(nodeIds[0]);
    const auto* n1 = mesh_.getNode(nodeIds[1]);
    const auto* n2 = mesh_.getNode(nodeIds[2]);
    const auto* n3 = mesh_.getNode(nodeIds[3]);
    return {n0->getCoordinates(), n1->getCoordinates(), n2->getCoordinates(), n3->getCoordinates()};
}

std::tuple<Point3D, Point3D, Point3D> ElementGeometry3D::getElementNodeCoordinates(const TriangleElement& element) const
{
    auto nodeIds = element.getNodeIds();
    const auto* n0 = mesh_.getNode(nodeIds[0]);
    const auto* n1 = mesh_.getNode(nodeIds[1]);
    const auto* n2 = mesh_.getNode(nodeIds[2]);
    return {n0->getCoordinates(), n1->getCoordinates(), n2->getCoordinates()};
}

} // namespace Meshing
