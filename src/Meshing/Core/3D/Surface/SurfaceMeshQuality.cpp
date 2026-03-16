#include "Meshing/Core/3D/Surface/SurfaceMeshQuality.h"

#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/Node2D.h"
#include "Meshing/Data/2D/TriangleElement.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace Meshing
{

namespace
{

// Lift the three UV-space nodes of a triangle to 3D via the surface.
std::array<Point3D, 3> getTrianglePoints3D(const TriangleElement& element,
                                            const MeshData2D& meshData,
                                            const Geometry3D::ISurface3D& surface)
{
    std::array<Point3D, 3> points;
    const auto& nodeIds = element.getNodeIdArray();
    for (std::size_t i = 0; i < 3; ++i)
    {
        const Node2D* node = meshData.getNode(nodeIds[i]);
        const auto& uv = node->getCoordinates();
        points[i] = surface.getPoint(uv.x(), uv.y());
    }
    return points;
}

// 3D area of a triangle given its three vertices.
double computeArea3D(const Point3D& a, const Point3D& b, const Point3D& c)
{
    return 0.5 * (b - a).cross(c - a).norm();
}

// Circumradius of the 3D triangle using R = (a * b * c) / (4 * Area).
// Returns 0 if the triangle is degenerate.
double computeCircumradius3D(const Point3D& a, const Point3D& b, const Point3D& c)
{
    const double edgeLengthAB = (b - a).norm();
    const double edgeLengthBC = (c - b).norm();
    const double edgeLengthCA = (a - c).norm();
    const double area = computeArea3D(a, b, c);
    if (area < 1e-15)
        return 0.0;
    return (edgeLengthAB * edgeLengthBC * edgeLengthCA) / (4.0 * area);
}

// Minimum interior angle of the 3D triangle (radians).
double computeMinAngle3D(const Point3D& a, const Point3D& b, const Point3D& c)
{
    const Point3D edgeAB = b - a;
    const Point3D edgeAC = c - a;
    const Point3D edgeBA = a - b;
    const Point3D edgeBC = c - b;
    const Point3D edgeCA = a - c;
    const Point3D edgeCB = b - c;

    auto clampedAngle = [](const Point3D& u, const Point3D& v) -> double
    {
        const double denominator = u.norm() * v.norm();
        if (denominator < 1e-15)
            return 0.0;
        const double cosAngle = std::clamp(u.dot(v) / denominator, -1.0, 1.0);
        return std::acos(cosAngle);
    };

    const double angleAtA = clampedAngle(edgeAB, edgeAC);
    const double angleAtB = clampedAngle(edgeBA, edgeBC);
    const double angleAtC = clampedAngle(edgeCA, edgeCB);

    return std::min({angleAtA, angleAtB, angleAtC});
}

} // namespace

SurfaceMeshQualityController::SurfaceMeshQualityController(
    const MeshData2D& meshData,
    const Geometry3D::ISurface3D& surface,
    double circumradiusToShortestEdgeRatioBound,
    double minAngleThresholdRadians,
    std::size_t elementLimit) :
    meshData_(meshData),
    surface_(surface),
    circumradiusToShortestEdgeRatioBound_(circumradiusToShortestEdgeRatioBound),
    minAngleThreshold_(minAngleThresholdRadians),
    elementLimit_(elementLimit)
{
}

bool SurfaceMeshQualityController::isMeshAcceptable(const MeshData2D& data) const
{
    if (data.getElementCount() >= elementLimit_)
        return true;

    for (const auto& [id, element] : data.getElements())
    {
        const auto* triangle = dynamic_cast<const TriangleElement*>(element.get());
        if (triangle == nullptr || !isTriangleAcceptable(*triangle))
            return false;
    }
    return true;
}

bool SurfaceMeshQualityController::isTriangleAcceptable(const TriangleElement& element) const
{
    const auto [a, b, c] = [&]() -> std::tuple<Point3D, Point3D, Point3D>
    {
        auto points = getTrianglePoints3D(element, meshData_, surface_);
        return {points[0], points[1], points[2]};
    }();

    const double edgeLengthAB = (b - a).norm();
    const double edgeLengthBC = (c - b).norm();
    const double edgeLengthCA = (a - c).norm();
    const double shortestEdge = std::min({edgeLengthAB, edgeLengthBC, edgeLengthCA});

    if (shortestEdge < 1e-10)
        return false;

    const double circumradius = computeCircumradius3D(a, b, c);
    if (circumradius / shortestEdge > circumradiusToShortestEdgeRatioBound_)
        return false;

    if (computeMinAngle3D(a, b, c) < minAngleThreshold_)
        return false;

    return true;
}

double SurfaceMeshQualityController::getTargetElementQuality() const
{
    return circumradiusToShortestEdgeRatioBound_;
}

std::size_t SurfaceMeshQualityController::getElementLimit() const
{
    return elementLimit_;
}

} // namespace Meshing
