#include "Meshing/Core/3D/RCDT/ProtectionExemption.h"

#include "Common/Types.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/Node3D.h"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace Meshing
{

namespace
{

// CGAL Mesh_3's thresholds (Facet_criterion_visitor_with_features): the angle
// criterion is waived below 0.5^2 * 4, the distance criterion below 0.1^2 * 4.
constexpr double ANGLE_EXEMPTION_RATIO = 1.0;
constexpr double CHORD_EXEMPTION_RATIO = 0.04;

struct WeightedPoint
{
    Point3D position;
    double weight;
};

/// Squared radius of the smallest sphere orthogonal to both weighted points.
/// Non-positive exactly when their balls intersect.
double squaredOrthogonalRadius(const WeightedPoint& a, const WeightedPoint& b)
{
    const double squaredDistance = (b.position - a.position).squaredNorm();
    const double t = (squaredDistance + a.weight - b.weight) / (2.0 * squaredDistance);
    return t * t * squaredDistance - a.weight;
}

/// Squared radius of the smallest sphere orthogonal to all three weighted
/// points: centred in their plane at equal power distance to each.
/// Non-positive exactly when the three balls have a common point.
double squaredOrthogonalRadius(const WeightedPoint& a, const WeightedPoint& b, const WeightedPoint& c)
{
    const Point3D u = b.position - a.position;
    const Point3D v = c.position - a.position;
    Eigen::Matrix2d system;
    system << u.dot(u), u.dot(v), u.dot(v), v.dot(v);
    const Eigen::Vector2d rightHandSide(0.5 * (u.dot(u) + a.weight - b.weight),
                                        0.5 * (v.dot(v) + a.weight - c.weight));
    if (std::abs(system.determinant()) < 1e-300)
        return std::numeric_limits<double>::max();
    const Eigen::Vector2d coefficients = system.inverse() * rightHandSide;
    return (coefficients(0) * u + coefficients(1) * v).squaredNorm() - a.weight;
}

} // namespace

ProtectionExemption protectionExemption(const TriangleElement& triangle, const MeshData3D& meshData)
{
    std::array<WeightedPoint, 3> weighted;
    std::array<WeightedPoint, 3> unweighted;
    size_t weightedCount = 0;
    size_t unweightedCount = 0;
    for (const size_t nodeId : triangle.getNodeIds())
    {
        const Node3D* node = meshData.getNode(nodeId);
        const WeightedPoint point{node->getCoordinates(), node->getWeight()};
        if (point.weight > 0.0)
            weighted[weightedCount++] = point;
        else
            unweighted[unweightedCount++] = point;
    }

    double ratio = 0.0;
    bool ballsIntersect = false;
    switch (weightedCount)
    {
    case 1:
        ratio = std::max(squaredOrthogonalRadius(weighted[0], unweighted[0]),
                         squaredOrthogonalRadius(weighted[0], unweighted[1])) /
                weighted[0].weight;
        break;
    case 2:
        ratio = std::max(squaredOrthogonalRadius(weighted[0], unweighted[0]) / weighted[0].weight,
                         squaredOrthogonalRadius(weighted[1], unweighted[0]) / weighted[1].weight);
        ballsIntersect = squaredOrthogonalRadius(weighted[0], weighted[1]) <= 0.0;
        break;
    case 3:
        ballsIntersect = squaredOrthogonalRadius(weighted[0], weighted[1], weighted[2]) <= 0.0;
        break;
    default:
        return {};
    }

    const bool anchoredToProtection = ballsIntersect || weightedCount == 1;
    return {anchoredToProtection && ratio < ANGLE_EXEMPTION_RATIO,
            anchoredToProtection && ratio < CHORD_EXEMPTION_RATIO};
}

} // namespace Meshing
