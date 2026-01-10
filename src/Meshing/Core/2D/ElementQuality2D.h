#pragma once

#include "Common/Types.h"
#include "Meshing/Core/2D/ElementGeometry2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include <optional>
#include <tuple>
#include <vector>

namespace Meshing
{

/// Helper that provides quality metrics for 2D mesh elements.
class ElementQuality2D
{
public:
    explicit ElementQuality2D(const MeshData2D& mesh);

    /// Computes the shortest edge length of a triangle element.
    double computeShortestEdgeLength(const TriangleElement& element) const;

    /// Computes the longest edge length of a triangle element.
    double computeLongestEdgeLength(const TriangleElement& element) const;

    /// Computes the ratio of circumradius to shortest edge length.
    /// Returns nullopt if the triangle is degenerate or has zero-length edges.
    std::optional<double> computeCircumradiusToShortestEdgeRatio(const TriangleElement& element) const;

    /// Returns triangle IDs sorted by quality (worst quality first).
    /// Quality is measured by circumradius-to-shortest-edge ratio (higher is worse).
    std::vector<size_t> getTrianglesSortedByQuality() const;

private:
    std::tuple<Point2D, Point2D, Point2D> getElementNodeCoordinates(const TriangleElement& element) const;

    const MeshData2D& mesh_;
    ElementGeometry2D geometry_;
};

} // namespace Meshing
