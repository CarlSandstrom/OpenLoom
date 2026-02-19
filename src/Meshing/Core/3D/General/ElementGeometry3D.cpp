#include "Meshing/Core/3D/General/ElementGeometry3D.h"
#include "Meshing/Core/3D/General/GeometryUtilities3D.h"
#include "Meshing/Data/2D/TriangleElement.h"

#include <Eigen/LU>
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

bool ElementGeometry3D::isPointInsideCircumscribingSphere(const TetrahedralElement& element,
                                                          const Point3D& point,
                                                          double tolerance) const
{
    if (const auto sphere = computeCircumscribingSphere(element))
    {
        return GeometryUtilities3D::isPointInsideCircumscribingSphere(*sphere, point, tolerance);
    }

    return false;
}

Point3D ElementGeometry3D::computeCentroid(const TetrahedralElement& element) const
{
    auto [v0, v1, v2, v3] = getElementNodeCoordinates(element);
    return Point3D((v0.x() + v1.x() + v2.x() + v3.x()) / 4.0,
                   (v0.y() + v1.y() + v2.y() + v3.y()) / 4.0,
                   (v0.z() + v1.z() + v2.z() + v3.z()) / 4.0);
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
