#include "Meshing/Core/3D/General/ElementGeometry3D.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"

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
    auto& nodeIds = element.getNodeIds();
    std::array<const Node3D*, 4> nodes;
    for (size_t i = 0; i < 4; ++i)
        nodes[i] = mesh_.getNode(nodeIds[i]);

    auto& origin = nodes[0]->getCoordinates();
    auto originWeight = nodes[0]->getWeight();

    // Solved for the offset from the origin:
    // 2 (c - origin) . (vi - origin) = |vi - origin|^2 - (wi - originWeight).
    Eigen::Matrix3d coefficientMatrix;
    Eigen::Vector3d rightHandSide;
    for (size_t i = 1; i < 4; ++i)
    {
        Point3D edge = nodes[i]->getCoordinates() - origin;
        coefficientMatrix.row(i - 1) = edge.transpose();
        rightHandSide(i - 1) = 0.5 * (edge.squaredNorm() - (nodes[i]->getWeight() - originWeight));
    }

    // A tetrahedron flat to rounding -- four points of a cyclic quad, such as
    // matching samples on the two circles bounding a cone -- has a whole line
    // of power-equidistant points normal to its plane. The minimum-norm offset
    // picks the one in the plane, so the element still has a dual vertex and
    // its faces a dual edge. Only a collinear triple leaves none.
    Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix3d> decomposition(coefficientMatrix);
    if (decomposition.rank() < 2)
    {
        return std::nullopt;
    }
    return Point3D(origin + decomposition.solve(rightHandSide));
}

std::optional<Point3D> ElementGeometry3D::computeOrthocenter(const TriangleElement& element) const
{
    auto& nodeIds = element.getNodeIds();
    auto firstNode = mesh_.getNode(nodeIds[0]);
    auto secondNode = mesh_.getNode(nodeIds[1]);
    auto thirdNode = mesh_.getNode(nodeIds[2]);

    // c = origin + a*u + b*v, with equal power distance to all three vertices:
    // 2 (c - origin) . u = |u|^2 + w0 - w1 (w = node weight), and likewise for v.
    auto& origin = firstNode->getCoordinates();
    Point3D u = secondNode->getCoordinates() - origin;
    Point3D v = thirdNode->getCoordinates() - origin;

    Eigen::Matrix2d coefficientMatrix;
    coefficientMatrix << u.dot(u), u.dot(v), u.dot(v), v.dot(v);
    Eigen::Vector2d rightHandSide(0.5 * (u.dot(u) + firstNode->getWeight() - secondNode->getWeight()),
                                  0.5 * (v.dot(v) + firstNode->getWeight() - thirdNode->getWeight()));

    Eigen::FullPivLU<Eigen::Matrix2d> decomposition(coefficientMatrix);
    if (!decomposition.isInvertible())
    {
        return std::nullopt;
    }
    Eigen::Vector2d coefficients = decomposition.solve(rightHandSide);
    return Point3D(origin + coefficients(0) * u + coefficients(1) * v);
}

} // namespace Meshing
