#pragma once

#include "Common/Types.h"
#include "Meshing/Data/MeshData3D.h"
#include "Meshing/Data/TetrahedralElement.h"
#include <optional>
#include <unordered_map>

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
    double getShortestEdgeLength(const TetrahedralElement& element) const;
    double getCircumradiusToShortestEdgeRatio(const TetrahedralElement& element) const;
    bool isSkinny(const TetrahedralElement& element, double threshold) const;

private:
    std::tuple<Point3D, Point3D, Point3D, Point3D> getElementNodeCoordinates(const TetrahedralElement& element) const;
    std::tuple<Point3D, Point3D, Point3D> getElementNodeCoordinates(const TriangleElement& element) const;

    const MeshData3D& mesh_;
};

} // namespace Meshing
