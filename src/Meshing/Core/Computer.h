#pragma once

#include <optional>
#include <unordered_map>

#include "Meshing/Core/ElementGeometry.h"
#include "Common/Types.h"

namespace Meshing
{

class TriangleElement;

/// Helper that owns mesh data and exposes computations that require node coordinates.
class Computer
{
public:
    struct CircumCircle2D
    {
        Point2D center;
        double radiusSquared;
    };

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

    // 2D circumcircle methods
    static std::optional<CircumCircle2D> computeCircumcircle(const TriangleElement& tri,
                                                              const std::unordered_map<size_t, Point2D>& nodeCoords);

    static bool isPointInsideCircumcircle(const CircumCircle2D& circle, const Point2D& point);

    double computeVolume(const TetrahedralElement& element) const;
    std::optional<ElementGeometry::CircumscribedSphere> getCircumscribingSphere(const TetrahedralElement& element) const;
    bool getIsPointInsideCircumscribingSphere(const TetrahedralElement& element, const Point3D& point) const;
    double computeArea(const TriangleElement& element) const;
    double computeQuality(const TetrahedralElement& element) const;
    double computeQuality(const TriangleElement& element) const;
    double getShortestEdgeLength(const TetrahedralElement& element) const;
    double getCircumradiusToShortestEdgeRatio(const TetrahedralElement& element) const;
    bool isSkinny(const TetrahedralElement& element, double threshold) const;

private:
    const MeshData& mesh_;
};

} // namespace Meshing
