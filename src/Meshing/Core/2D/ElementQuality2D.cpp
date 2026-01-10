#include "ElementQuality2D.h"
#include "GeometryUtilities2D.h"

#include <algorithm>
#include <limits>

namespace Meshing
{

ElementQuality2D::ElementQuality2D(const MeshData2D& mesh) :
    mesh_(mesh),
    geometry_(mesh)
{
}

double ElementQuality2D::computeShortestEdgeLength(const TriangleElement& element) const
{
    auto [p0, p1, p2] = getElementNodeCoordinates(element);
    const double e0 = GeometryUtilities2D::computeEdgeLength(p0, p1);
    const double e1 = GeometryUtilities2D::computeEdgeLength(p1, p2);
    const double e2 = GeometryUtilities2D::computeEdgeLength(p2, p0);
    return std::min({e0, e1, e2});
}

double ElementQuality2D::computeLongestEdgeLength(const TriangleElement& element) const
{
    auto [p0, p1, p2] = getElementNodeCoordinates(element);
    const double e0 = GeometryUtilities2D::computeEdgeLength(p0, p1);
    const double e1 = GeometryUtilities2D::computeEdgeLength(p1, p2);
    const double e2 = GeometryUtilities2D::computeEdgeLength(p2, p0);
    return std::max({e0, e1, e2});
}

std::optional<double> ElementQuality2D::computeCircumradiusToShortestEdgeRatio(const TriangleElement& element) const
{
    auto circumcircle = geometry_.computeCircumcircle(element);
    if (!circumcircle.has_value())
    {
        return std::nullopt;
    }
    const double shortestEdge = computeShortestEdgeLength(element);
    if (shortestEdge < 1e-10)
    {
        return std::nullopt;
    }
    return circumcircle->radius / shortestEdge;
}

std::vector<size_t> ElementQuality2D::getTrianglesSortedByQuality() const
{
    std::vector<std::pair<size_t, double>> trianglesWithQuality;

    for (const auto& [elementId, element] : mesh_.getElements())
    {
        const auto* triangle = dynamic_cast<const TriangleElement*>(element.get());
        if (triangle == nullptr)
        {
            continue;
        }

        auto ratio = computeCircumradiusToShortestEdgeRatio(*triangle);
        double quality = ratio.has_value() ? ratio.value() : std::numeric_limits<double>::max();
        trianglesWithQuality.emplace_back(elementId, quality);
    }

    std::sort(trianglesWithQuality.begin(), trianglesWithQuality.end(),
              [](const auto& a, const auto& b) { return a.second > b.second; });

    std::vector<size_t> result;
    result.reserve(trianglesWithQuality.size());
    for (const auto& [id, quality] : trianglesWithQuality)
    {
        result.push_back(id);
    }

    return result;
}

std::tuple<Point2D, Point2D, Point2D> ElementQuality2D::getElementNodeCoordinates(const TriangleElement& element) const
{
    auto nodeIds = element.getNodeIds();
    const auto* n0 = mesh_.getNode(nodeIds[0]);
    const auto* n1 = mesh_.getNode(nodeIds[1]);
    const auto* n2 = mesh_.getNode(nodeIds[2]);
    return {n0->getCoordinates(), n1->getCoordinates(), n2->getCoordinates()};
}

} // namespace Meshing
