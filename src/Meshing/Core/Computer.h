#pragma once

#include <optional>
#include <unordered_map>

#include "Common/Types.h"
#include "Meshing/Core/ElementGeometry.h"

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

    struct CircumscribedSphere
    {
        Point3D center;
        double radius;
    };

    explicit Computer(const MeshData3D& mesh);

    double computeVolume(const TetrahedralElement& element);

    double computeArea(const TriangleElement& element) const;

    std::optional<CircumscribedSphere> computeCircumscribingSphere(const TetrahedralElement& element) const;

    static bool getIsPointInsideCircumscribingSphere(const CircumscribedSphere& sphere,
                                                     const Point3DRef point,
                                                     double tolerance = 1e-12);

    static std::optional<CircumCircle2D> computeCircumcircle(const TriangleElement& tri,
                                                             const std::unordered_map<size_t, Point2D>& nodeCoords);

    static bool isPointInsideCircumcircle(const CircumCircle2D& circle, const Point2D& point);

    bool getIsPointInsideCircumscribingSphere(const TetrahedralElement& element, const Point3D& point) const;
    double computeQuality(const TetrahedralElement& element) const;
    double computeQuality(const TriangleElement& element) const;
    double getShortestEdgeLength(const TetrahedralElement& element) const;
    double getCircumradiusToShortestEdgeRatio(const TetrahedralElement& element) const;
    bool isSkinny(const TetrahedralElement& element, double threshold) const;

private:
    const MeshData3D& mesh_;
};

} // namespace Meshing
