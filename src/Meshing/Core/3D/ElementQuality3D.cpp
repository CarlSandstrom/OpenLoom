#include "ElementQuality3D.h"

#include <algorithm>
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

} // namespace Meshing
