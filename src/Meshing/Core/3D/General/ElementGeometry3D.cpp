#include "Meshing/Core/3D/General/ElementGeometry3D.h"
#include "Meshing/Data/2D/TriangleElement.h"

#include <Eigen/LU>
#include <Eigen/QR>
#include <array>
#include <cmath>

namespace Meshing
{

ElementGeometry3D::ElementGeometry3D(const MeshData3D& mesh) :
    mesh_(mesh)
{
}

std::optional<Point3D> ElementGeometry3D::computeOrthocenter(const TetrahedralElement& element) const
{
    const auto& nodeIds = element.getNodeIds();
    std::array<const Node3D*, 4> nodes;
    for (size_t i = 0; i < 4; ++i)
        nodes[i] = mesh_.getNode(nodeIds[i]);

    const Point3D& v0 = nodes[0]->getCoordinates();
    const double w0 = nodes[0]->getWeight();

    // Solved for the offset from v0: 2 (c - v0) . (vi - v0) = |vi - v0|^2 - (wi - w0).
    Eigen::Matrix3d A;
    Eigen::Vector3d b;
    for (size_t i = 1; i < 4; ++i)
    {
        const Point3D edge = nodes[i]->getCoordinates() - v0;
        A.row(i - 1) = edge.transpose();
        b(i - 1) = 0.5 * (edge.squaredNorm() - (nodes[i]->getWeight() - w0));
    }

    // A tetrahedron flat to rounding -- four points of a cyclic quad, such as
    // matching samples on the two circles bounding a cone -- has a whole line
    // of power-equidistant points normal to its plane. The minimum-norm offset
    // picks the one in the plane, so the element still has a dual vertex and
    // its faces a dual edge. Only a collinear triple leaves none.
    const Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix3d> decomposition(A);
    if (decomposition.rank() < 2)
    {
        return std::nullopt;
    }
    return Point3D(v0 + decomposition.solve(b));
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

} // namespace Meshing
