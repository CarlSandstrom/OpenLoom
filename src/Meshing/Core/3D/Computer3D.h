#pragma once

#include "Common/Types.h"
#include "ElementGeometry3D.h"
#include "ElementQuality3D.h"
#include "GeometryStructures3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"
#include <optional>

namespace Meshing
{

class TriangleElement;

/// Helper that owns 3D mesh data and exposes computations that require node coordinates.
/// NOTE: This class is a facade that delegates to focused helper classes.
/// Consider using ElementGeometry3D, ElementQuality3D, or GeometryUtilities3D directly.
class Computer3D
{
public:
    explicit Computer3D(const MeshData3D& mesh);

    // Methods delegated to ElementGeometry3D
    double computeVolume(const TetrahedralElement& element) const;
    double computeArea(const TriangleElement& element) const;
    std::optional<CircumscribedSphere> computeCircumscribingSphere(const TetrahedralElement& element) const;
    bool getIsPointInsideCircumscribingSphere(const TetrahedralElement& element, const Point3D& point, double tolerance = 1e-12) const;

    // Methods delegated to ElementQuality3D
    double getShortestEdgeLength(const TetrahedralElement& element) const;
    double getCircumradiusToShortestEdgeRatio(const TetrahedralElement& element) const;
    bool isSkinny(const TetrahedralElement& element, double threshold) const;

private:
    const MeshData3D& mesh_;
    ElementGeometry3D geometry_;
    ElementQuality3D quality_;
};

} // namespace Meshing
