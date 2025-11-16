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

    static double computeVolume(const Point3DRef v0,
                                const Point3DRef v1,
                                const Point3DRef v2,
                                const Point3DRef v3);

    static double computeArea(const Point3DRef v0,
                              const Point3DRef v1,
                              const Point3DRef v2);

    static std::optional<ElementGeometry::CircumscribedSphere> getCircumscribingSphere(const Point3DRef v0,
                                                                                       const Point3DRef v1,
                                                                                       const Point3DRef v2,
                                                                                       const Point3DRef v3);

    static bool getIsPointInsideCircumscribingSphere(const ElementGeometry::CircumscribedSphere& sphere,
                                                     const Point3DRef point,
                                                     double tolerance = 1e-12);

    double computeVolume(const TetrahedralElement& element) const;
    std::optional<ElementGeometry::CircumscribedSphere> getCircumscribingSphere(const TetrahedralElement& element) const;
    bool getIsPointInsideCircumscribingSphere(const TetrahedralElement& element, const Point3D& point) const;
    double computeArea(const TriangleElement& element) const;
    double computeQuality(const TetrahedralElement& element) const;
    double computeQuality(const TriangleElement& element) const;

private:
    const MeshData& mesh_;
};

} // namespace Meshing
