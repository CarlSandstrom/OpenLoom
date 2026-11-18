#include "Meshing/Core/3D/General/ElementGeometry3D.h"
#include "Meshing/Core/3D/General/ExactArithmetic3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/TetrahedronElement.h"
#include "Meshing/Data/Base/TriangleElement.h"

#include <Eigen/LU>
#include <Eigen/QR>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>

namespace Meshing
{

namespace
{

using TetrahedronNodes = std::array<const Node3D*, 4>;

// A floating-point offset is used only when its certified error is within this fraction (2^-40)
// of the offset's length. A sliver flat to within about that much falls back to the exact path;
// OPE-214's, wrong by 1.4e-5 of its length in a floating-point solve, is far outside it.
constexpr double CERTIFIED_RELATIVE_ACCURACY = 0x1p-40;

// Bounds on the floating-point error of a 3x3 determinant evaluated by cofactor expansion, as a
// multiple of machine epsilon times the permanent of the absolute entries: one for entries that
// are rounded differences, and a larger one when a column holds the rounded right-hand side.
constexpr double DENOMINATOR_ERROR_FACTOR = 10.0;
constexpr double NUMERATOR_ERROR_FACTOR = 20.0;

// The bound on the rounded right-hand side |e|^2 - (wi - w0), as a multiple of machine epsilon
// times |e|^2 + |wi - w0|.
constexpr double RIGHT_HAND_SIDE_ERROR_FACTOR = 8.0;

std::optional<Eigen::Vector3d> solveCertifiedOrthocenterOffset(const TetrahedronNodes& nodes);
std::optional<Eigen::Vector3d> solveOrthocenterOffsetExactly(const TetrahedronNodes& nodes);
std::optional<Eigen::Vector3d> solveFlatOrthocenterOffset(const TetrahedronNodes& nodes);

} // namespace

ElementGeometry3D::ElementGeometry3D(const MeshData3D& mesh) :
    mesh_(mesh)
{
}

std::optional<Point3D> ElementGeometry3D::computeOrthocenter(const TetrahedronElement& element) const
{
    auto& nodeIds = element.getNodeIds();
    TetrahedronNodes nodes;
    for (size_t i = 0; i < 4; ++i)
        nodes[i] = mesh_.getNode(nodeIds[i]);

    auto offset = solveCertifiedOrthocenterOffset(nodes);
    if (!offset) offset = solveOrthocenterOffsetExactly(nodes);
    if (!offset) offset = solveFlatOrthocenterOffset(nodes);
    if (!offset) return std::nullopt;
    return Point3D(nodes[0]->getCoordinates() + *offset);
}

double ElementGeometry3D::computeMinimumDihedralAngle(const std::array<Point3D, 4>& points)
{
    double minimum = 180.0;
    for (int i = 0; i < 4; ++i)
        for (int j = i + 1; j < 4; ++j)
        {
            int k = 0;
            while (k == i || k == j) ++k;
            int l = 6 - i - j - k;
            Vector3D edge = points[j] - points[i];
            Vector3D normal0 = edge.cross(points[k] - points[i]);
            Vector3D normal1 = edge.cross(points[l] - points[i]);
            auto denominator = normal0.norm() * normal1.norm();
            if (!(denominator > 0.0)) return 0.0;
            auto cosine = std::clamp(normal0.dot(normal1) / denominator, -1.0, 1.0);
            minimum = std::min(minimum, std::acos(cosine) * 180.0 / std::numbers::pi);
        }
    return minimum;
}

std::optional<Point3D> ElementGeometry3D::computeOrthocenter(const TriangleElement& element) const
{
    auto& nodeIds = element.getNodeIds();
    auto node0 = mesh_.getNode(nodeIds[0]);
    auto node1 = mesh_.getNode(nodeIds[1]);
    auto node2 = mesh_.getNode(nodeIds[2]);

    // c = origin + a*u + b*v, with equal power distance to all three vertices:
    // 2 (c - origin) . u = |u|^2 + w0 - w1 (w = node weight), and likewise for v.
    auto& origin = node0->getCoordinates();
    Point3D u = node1->getCoordinates() - origin;
    Point3D v = node2->getCoordinates() - origin;

    Eigen::Matrix2d coefficientMatrix;
    coefficientMatrix << u.dot(u), u.dot(v), u.dot(v), v.dot(v);
    Eigen::Vector2d rightHandSide(0.5 * (u.dot(u) + node0->getWeight() - node1->getWeight()),
                                  0.5 * (v.dot(v) + node0->getWeight() - node2->getWeight()));

    Eigen::FullPivLU<Eigen::Matrix2d> decomposition(coefficientMatrix);
    if (!decomposition.isInvertible())
    {
        return std::nullopt;
    }
    Eigen::Vector2d coefficients = decomposition.solve(rightHandSide);
    return Point3D(origin + coefficients(0) * u + coefficients(1) * v);
}

namespace
{

double permanent(const Eigen::Matrix3d& matrix)
{
    return matrix(0, 0) * (matrix(1, 1) * matrix(2, 2) + matrix(1, 2) * matrix(2, 1)) +
           matrix(0, 1) * (matrix(1, 0) * matrix(2, 2) + matrix(1, 2) * matrix(2, 0)) +
           matrix(0, 2) * (matrix(1, 0) * matrix(2, 1) + matrix(1, 1) * matrix(2, 0));
}

double cofactorDeterminant(const Eigen::Matrix3d& matrix)
{
    return matrix(0, 0) * (matrix(1, 1) * matrix(2, 2) - matrix(1, 2) * matrix(2, 1)) -
           matrix(0, 1) * (matrix(1, 0) * matrix(2, 2) - matrix(1, 2) * matrix(2, 0)) +
           matrix(0, 2) * (matrix(1, 0) * matrix(2, 1) - matrix(1, 1) * matrix(2, 0));
}

// The offset by Cramer's rule in doubles, with an a-priori bound on its rounding error (Shewchuk's
// style of forward error analysis). nullopt when the bound cannot certify it, which is what a
// sliver's tiny determinant does: the caller then solves exactly.
std::optional<Eigen::Vector3d> solveCertifiedOrthocenterOffset(const TetrahedronNodes& nodes)
{
    auto epsilon = std::numeric_limits<double>::epsilon();
    auto& origin = nodes[0]->getCoordinates();
    Eigen::Matrix3d edges;
    Eigen::Vector3d rightHandSide;
    Eigen::Vector3d rightHandSideMagnitude;
    for (int i = 0; i < 3; ++i)
    {
        edges.row(i) = (nodes[i + 1]->getCoordinates() - origin).transpose();
        auto weightDifference = nodes[i + 1]->getWeight() - nodes[0]->getWeight();
        rightHandSide(i) = edges.row(i).squaredNorm() - weightDifference;
        rightHandSideMagnitude(i) = edges.row(i).squaredNorm() + std::abs(weightDifference);
    }

    Eigen::Matrix3d absoluteEdges = edges.cwiseAbs();
    auto denominator = cofactorDeterminant(edges);
    auto denominatorError = DENOMINATOR_ERROR_FACTOR * epsilon * permanent(absoluteEdges);
    if (std::abs(denominator) <= denominatorError) return std::nullopt;

    Eigen::Vector3d offset;
    Eigen::Vector3d offsetError;
    for (int column = 0; column < 3; ++column)
    {
        Eigen::Matrix3d numeratorMatrix = edges;
        numeratorMatrix.col(column) = rightHandSide;
        Eigen::Matrix3d numeratorMagnitude = absoluteEdges;
        numeratorMagnitude.col(column) = rightHandSideMagnitude;
        auto numeratorError = (NUMERATOR_ERROR_FACTOR + RIGHT_HAND_SIDE_ERROR_FACTOR) * epsilon *
                              permanent(numeratorMagnitude);

        offset(column) = cofactorDeterminant(numeratorMatrix) / (2.0 * denominator);
        offsetError(column) = (0.5 * numeratorError + std::abs(offset(column)) * denominatorError) /
                                  (std::abs(denominator) - denominatorError) +
                              epsilon * std::abs(offset(column));
    }
    if (offsetError.norm() > CERTIFIED_RELATIVE_ACCURACY * offset.norm()) return std::nullopt;
    return offset;
}

// The offset c - origin from 2 (c - origin) . (vi - origin) = |vi - origin|^2 - (wi - originWeight),
// by Cramer's rule with every determinant exact, so only the final division rounds. A sliver's
// system is too badly conditioned for a floating-point solve: on OPE-214's flat sliver, Eigen's QR
// put the orthocenter exactly in the surface it lies 4.8e-5 above. nullopt when the tetrahedron is
// exactly flat.
std::optional<Eigen::Vector3d> solveOrthocenterOffsetExactly(const TetrahedronNodes& nodes)
{
    using Expansion = ExactArithmetic3D::Expansion;

    auto& origin = nodes[0]->getCoordinates();
    Expansion edges[3][3];
    Expansion rightHandSide[3];
    for (size_t i = 0; i < 3; ++i)
    {
        auto& vertex = nodes[i + 1]->getCoordinates();
        for (int k = 0; k < 3; ++k)
            edges[i][k] = ExactArithmetic3D::expansionFromDifference(vertex(k), origin(k));
        rightHandSide[i] = ExactArithmetic3D::expansionSub(
            ExactArithmetic3D::squaredNormRelative(edges[i][0], edges[i][1], edges[i][2]),
            ExactArithmetic3D::expansionFromDifference(nodes[i + 1]->getWeight(), nodes[0]->getWeight()));
    }

    auto denominator = ExactArithmetic3D::det3x3(edges);
    if (ExactArithmetic3D::expansionSign(denominator) == 0) return std::nullopt;
    auto twiceDenominator = 2.0 * ExactArithmetic3D::estimate(denominator);

    Eigen::Vector3d offset;
    for (int column = 0; column < 3; ++column)
    {
        Expansion numeratorMatrix[3][3];
        for (size_t i = 0; i < 3; ++i)
            for (int k = 0; k < 3; ++k)
                numeratorMatrix[i][k] = k == column ? rightHandSide[i] : edges[i][k];
        offset(column) = ExactArithmetic3D::estimate(ExactArithmetic3D::det3x3(numeratorMatrix)) / twiceDenominator;
    }
    return offset;
}

// An exactly flat tetrahedron -- four points of a cyclic quad -- has a whole line of
// power-equidistant points normal to its plane. The minimum-norm offset picks the one in the
// plane, so the element still has a dual vertex and its faces a dual edge. Only a collinear
// triple leaves none.
std::optional<Eigen::Vector3d> solveFlatOrthocenterOffset(const TetrahedronNodes& nodes)
{
    auto& origin = nodes[0]->getCoordinates();
    Eigen::Matrix3d coefficientMatrix;
    Eigen::Vector3d rightHandSide;
    for (size_t i = 1; i < 4; ++i)
    {
        Point3D edge = nodes[i]->getCoordinates() - origin;
        coefficientMatrix.row(i - 1) = edge.transpose();
        rightHandSide(i - 1) = 0.5 * (edge.squaredNorm() - (nodes[i]->getWeight() - nodes[0]->getWeight()));
    }

    Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix3d> decomposition(coefficientMatrix);
    if (decomposition.rank() < 2) return std::nullopt;
    return Eigen::Vector3d(decomposition.solve(rightHandSide));
}

} // namespace

} // namespace Meshing
