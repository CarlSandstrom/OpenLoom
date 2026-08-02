#include "Meshing/Core/3D/General/ElementQuality3D.h"

#include "Meshing/Data/2D/TriangleElement.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace Meshing
{

ElementQuality3D::ElementQuality3D(const MeshData3D& mesh) :
    mesh_(mesh),
    geometry_(mesh)
{
}

double ElementQuality3D::getShortestEdgeLength(const TetrahedralElement& element) const
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

double ElementQuality3D::getCircumradiusToShortestEdgeRatio(const TetrahedralElement& element) const
{
    constexpr double MIN_EDGE = 1e-15;

    const auto sphere = geometry_.computeCircumscribingSphere(element);
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

bool ElementQuality3D::isSkinny(const TetrahedralElement& element, double threshold) const
{
    const double ratio = getCircumradiusToShortestEdgeRatio(element);
    if (ratio == 0.0)
    {
        return false;
    }

    return ratio > threshold;
}

std::vector<std::pair<size_t, double>> ElementQuality3D::getSkinnyTetrahedraSortedByQuality(double threshold) const
{
    std::vector<std::pair<size_t, double>> result;

    for (const auto& [tetId, element] : mesh_.getElements())
    {
        const auto* tet = dynamic_cast<const TetrahedralElement*>(element.get());
        if (!tet)
        {
            continue;
        }

        double ratio = getCircumradiusToShortestEdgeRatio(*tet);

        // Skip invalid ratios
        if (!std::isfinite(ratio) || ratio <= 0.0)
        {
            continue;
        }

        if (ratio > threshold)
        {
            result.emplace_back(tetId, ratio);
        }
    }

    // Sort by ratio descending (worst quality first)
    std::sort(result.begin(), result.end(),
              [](const auto& a, const auto& b)
              { return a.second > b.second; });

    return result;
}

double ElementQuality3D::getShortestEdgeLength(const TriangleElement& element) const
{
    const auto& nodeIds = element.getNodeIdArray();

    double minLen = std::numeric_limits<double>::max();
    for (size_t i = 0; i < 3; ++i)
    {
        const auto* n1 = mesh_.getNode(nodeIds[i]);
        const auto* n2 = mesh_.getNode(nodeIds[(i + 1) % 3]);
        if (!n1 || !n2)
            return 0.0;

        minLen = std::min(minLen, (n1->getCoordinates() - n2->getCoordinates()).norm());
    }

    return minLen;
}

double ElementQuality3D::getMinAngle(const TriangleElement& element) const
{
    const auto& nodeIds = element.getNodeIdArray();
    const auto* n0 = mesh_.getNode(nodeIds[0]);
    const auto* n1 = mesh_.getNode(nodeIds[1]);
    const auto* n2 = mesh_.getNode(nodeIds[2]);
    if (!n0 || !n1 || !n2)
        return 0.0;

    const Point3D& p0 = n0->getCoordinates();
    const Point3D& p1 = n1->getCoordinates();
    const Point3D& p2 = n2->getCoordinates();

    auto clampedAngle = [](const Point3D& u, const Point3D& v) -> double
    {
        const double denominator = u.norm() * v.norm();
        if (denominator < 1e-15)
            return 0.0;
        const double cosAngle = std::clamp(u.dot(v) / denominator, -1.0, 1.0);
        return std::acos(cosAngle);
    };

    const double angleAt0 = clampedAngle(p1 - p0, p2 - p0);
    const double angleAt1 = clampedAngle(p0 - p1, p2 - p1);
    const double angleAt2 = clampedAngle(p0 - p2, p1 - p2);

    return std::min({angleAt0, angleAt1, angleAt2});
}

} // namespace Meshing
