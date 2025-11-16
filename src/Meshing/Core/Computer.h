#pragma once

#include <optional>

#include "Meshing/Core/ElementGeometry.h"

namespace Meshing
{

/// Helper that owns mesh data and exposes computations that require node coordinates.
class Computer
{
public:
    explicit Computer(const MeshData& mesh);

    double computeVolume(const TetrahedralElement& element) const;
    std::optional<ElementGeometry::CircumscribedSphere> getCircumscribingSphere(const TetrahedralElement& element) const;
    bool getIsPointInsideCircumscribingSphere(const TetrahedralElement& element, const Point3D& point) const;

private:
    const MeshData& mesh_;
};

} // namespace Meshing
