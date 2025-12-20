#pragma once

#include "Common/Types.h"
#include "ComputerGeneral.h"
#include "Meshing/Data/MeshData3D.h"
#include "Meshing/Data/TetrahedralElement.h"
#include <optional>

namespace Meshing
{

class TriangleElement;

/// Helper that owns 3D mesh data and exposes computations that require node coordinates.
class Computer3D
{
public:
    using CircumscribedSphere = ComputerGeneral::CircumscribedSphere;

    explicit Computer3D(const MeshData3D& mesh);

    double computeVolume(const TetrahedralElement& element) const;

    double computeArea(const TriangleElement& element) const;

    std::optional<CircumscribedSphere> computeCircumscribingSphere(const TetrahedralElement& element) const;

    bool getIsPointInsideCircumscribingSphere(const TetrahedralElement& element, const Point3D& point, double tolerance = 1e-12) const;

    double getShortestEdgeLength(const TetrahedralElement& element) const;

    double getCircumradiusToShortestEdgeRatio(const TetrahedralElement& element) const;

    bool isSkinny(const TetrahedralElement& element, double threshold) const;

private:
    std::tuple<Point3D, Point3D, Point3D, Point3D> getElementNodeCoordinates(const TetrahedralElement& element) const;
    std::tuple<Point3D, Point3D, Point3D> getElementNodeCoordinates(const TriangleElement& element) const;

    const MeshData3D& mesh_;
};

} // namespace Meshing
