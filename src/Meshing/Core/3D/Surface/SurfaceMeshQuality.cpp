#include "Meshing/Core/3D/Surface/SurfaceMeshQuality.h"

#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/Node2D.h"
#include "Meshing/Data/2D/TriangleElement.h"

#include <algorithm>
#include <cmath>

namespace Meshing
{

namespace
{

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
    std::size_t elementLimit,
    double chordDeviationTolerance) :
    meshData_(meshData),
    surface_(surface),
    circumradiusToShortestEdgeRatioBound_(circumradiusToShortestEdgeRatioBound),
    minAngleThreshold_(minAngleThresholdRadians),
    elementLimit_(elementLimit),
    chordDeviationTolerance_(chordDeviationTolerance)
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
    const auto& nodeIds = element.getNodeIdArray();
    const auto& uvA = meshData_.getNode(nodeIds[0])->getCoordinates();
    const auto& uvB = meshData_.getNode(nodeIds[1])->getCoordinates();
    const auto& uvC = meshData_.getNode(nodeIds[2])->getCoordinates();

    const Point3D vertexA = surface_.getPoint(uvA.x(), uvA.y());
    const Point3D vertexB = surface_.getPoint(uvB.x(), uvB.y());
    const Point3D vertexC = surface_.getPoint(uvC.x(), uvC.y());

    const double edgeLengthAB = (vertexB - vertexA).norm();
    const double edgeLengthBC = (vertexC - vertexB).norm();
    const double edgeLengthCA = (vertexA - vertexC).norm();
    const double shortestEdge = std::min({edgeLengthAB, edgeLengthBC, edgeLengthCA});

    if (shortestEdge < 1e-10)
        return false;

    const double circumradius = computeCircumradius3D(vertexA, vertexB, vertexC);
    if (circumradius / shortestEdge > circumradiusToShortestEdgeRatioBound_)
        return false;

    if (computeMinAngle3D(vertexA, vertexB, vertexC) < minAngleThreshold_)
        return false;

    if (chordDeviationTolerance_ > 0.0)
    {
        // Check chord deviation: compare the flat triangle against the actual CAD surface
        // at the centroid and three edge midpoints. Reject if any exceeds the tolerance.
        auto chordDeviation = [&](double u, double v, const Point3D& flatPoint) -> double
        {
            return (surface_.getPoint(u, v) - flatPoint).norm();
        };

        // Centroid
        const Point3D flatCentroid = (vertexA + vertexB + vertexC) / 3.0;
        if (chordDeviation((uvA.x() + uvB.x() + uvC.x()) / 3.0,
                           (uvA.y() + uvB.y() + uvC.y()) / 3.0,
                           flatCentroid) > chordDeviationTolerance_)
            return false;

        // Edge midpoints
        if (chordDeviation((uvA.x() + uvB.x()) * 0.5, (uvA.y() + uvB.y()) * 0.5,
                           (vertexA + vertexB) * 0.5) > chordDeviationTolerance_)
            return false;

        if (chordDeviation((uvB.x() + uvC.x()) * 0.5, (uvB.y() + uvC.y()) * 0.5,
                           (vertexB + vertexC) * 0.5) > chordDeviationTolerance_)
            return false;

        if (chordDeviation((uvC.x() + uvA.x()) * 0.5, (uvC.y() + uvA.y()) * 0.5,
                           (vertexC + vertexA) * 0.5) > chordDeviationTolerance_)
            return false;
    }

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
