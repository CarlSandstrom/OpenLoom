#pragma once

#include <optional>

#include "Common/Types.h"
#include "Meshing/Data/MeshData.h"
#include "Meshing/Data/TetrahedralElement.h"
#include "Meshing/Data/TriangleElement.h"

namespace Meshing::ElementGeometry
{

struct CircumscribedSphere
{
    Point3D center;
    double radius;
};

double computeVolume(const Point3DRef v0,
                     const Point3DRef v1,
                     const Point3DRef v2,
                     const Point3DRef v3);

std::optional<double> computeVolume(const MeshData& mesh, const TetrahedralElement& element);

double computeArea(const Point3DRef v0,
                   const Point3DRef v1,
                   const Point3DRef v2);

std::optional<double> computeArea(const MeshData& mesh, const TriangleElement& element);

std::optional<CircumscribedSphere> computeCircumscribingSphere(const Point3DRef v0,
                                                               const Point3DRef v1,
                                                               const Point3DRef v2,
                                                               const Point3DRef v3);

std::optional<CircumscribedSphere> computeCircumscribingSphere(const MeshData& mesh,
                                                               const TetrahedralElement& element);

std::optional<double> computeQuality(const MeshData& mesh, const TetrahedralElement& element);

std::optional<double> computeQuality(const MeshData& mesh, const TriangleElement& element);

} // namespace Meshing::ElementGeometry
