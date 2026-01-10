#pragma once

#include "Common/Types.h"
#include "Meshing/Core/3D/ElementGeometry3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"

namespace Meshing
{

/// Helper that provides quality metrics for 3D mesh elements.
class ElementQuality3D
{
public:
    explicit ElementQuality3D(const MeshData3D& mesh);

    /// Computes the shortest edge length of a tetrahedral element.
    double getShortestEdgeLength(const TetrahedralElement& element) const;

    /// Computes the ratio of circumradius to shortest edge length.
    /// Returns 0.0 if the element is degenerate.
    /// Returns infinity if the shortest edge is too small.
    double getCircumradiusToShortestEdgeRatio(const TetrahedralElement& element) const;

    /// Tests if a tetrahedral element is skinny based on the circumradius-to-edge ratio.
    bool isSkinny(const TetrahedralElement& element, double threshold) const;

private:
    const MeshData3D& mesh_;
    ElementGeometry3D geometry_;
};

} // namespace Meshing
