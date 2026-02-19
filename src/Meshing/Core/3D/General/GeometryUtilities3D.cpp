#include "Meshing/Core/3D/General/GeometryUtilities3D.h"
#include <cmath>

namespace Meshing
{

bool GeometryUtilities3D::isPointInsideCircumscribingSphere(const CircumscribedSphere& sphere,
                                                            const Point3DRef point,
                                                            double tolerance)
{
    const double radiusSq = sphere.radius * sphere.radius;
    const double distanceSq = (point - sphere.center).squaredNorm();
    return distanceSq <= radiusSq + tolerance;
}

double GeometryUtilities3D::computeEdgeLength(const Point3D& p1, const Point3D& p2)
{
    return (p2 - p1).norm();
}

DiametralSphere GeometryUtilities3D::createDiametralSphere(const Point3D& p1, const Point3D& p2)
{
    DiametralSphere sphere;
    sphere.center = (p1 + p2) * 0.5;
    sphere.radius = computeEdgeLength(p1, p2) * 0.5;
    return sphere;
}

bool GeometryUtilities3D::isPointInDiametralSphere(const DiametralSphere& sphere,
                                                   const Point3D& point,
                                                   double tolerance)
{
    const double radiusSq = sphere.radius * sphere.radius;
    const double distanceSq = (point - sphere.center).squaredNorm();
    // Point is inside if distance to center is less than radius
    // Use strict inequality to exclude points exactly on the sphere
    return distanceSq < radiusSq - tolerance;
}

EquatorialSphere GeometryUtilities3D::createEquatorialSphere(const Point3D& p1,
                                                             const Point3D& p2,
                                                             const Point3D& p3)
{
    // The equatorial sphere passes through three points of a triangle
    // Its center is the circumcenter of the triangle (lies in the triangle's plane)

    // Compute edge vectors
    Point3D v1 = p2 - p1;
    Point3D v2 = p3 - p1;

    // Compute normal to the plane
    Point3D normal = v1.cross(v2);
    double normalLengthSq = normal.squaredNorm();

    // Check for degenerate triangle
    if (normalLengthSq < 1e-24)
    {
        // Triangle is degenerate, return sphere at centroid with zero radius
        EquatorialSphere sphere;
        sphere.center = (p1 + p2 + p3) / 3.0;
        sphere.radius = 0.0;
        return sphere;
    }

    // Compute circumcenter using the formula:
    // center = p1 + (|v2|^2 * (normal x v1) + |v1|^2 * (v2 x normal)) / (2 * |normal|^2)
    double v1LengthSq = v1.squaredNorm();
    double v2LengthSq = v2.squaredNorm();

    Point3D term1 = normal.cross(v1) * v2LengthSq;
    Point3D term2 = v2.cross(normal) * v1LengthSq;

    EquatorialSphere sphere;
    sphere.center = p1 + (term1 + term2) / (2.0 * normalLengthSq);
    sphere.radius = computeEdgeLength(sphere.center, p1);

    return sphere;
}

bool GeometryUtilities3D::isPointInEquatorialSphere(const EquatorialSphere& sphere,
                                                    const Point3D& point,
                                                    const Point3D& triangleP1,
                                                    const Point3D& triangleP2,
                                                    const Point3D& triangleP3,
                                                    double tolerance)
{
    // First check if point is coplanar with the triangle
    // If coplanar, it doesn't encroach (by definition in Shewchuk's paper)
    Point3D v1 = triangleP2 - triangleP1;
    Point3D v2 = triangleP3 - triangleP1;
    Point3D normal = v1.cross(v2);

    // Check coplanarity: dot product of (point - p1) with normal should be ~0
    Point3D toPoint = point - triangleP1;
    double dotProduct = std::abs(toPoint.dot(normal));
    double normalLength = normal.norm();

    if (dotProduct < tolerance * normalLength)
    {
        // Point is coplanar, not encroaching
        return false;
    }

    // Point is non-coplanar, check if it's inside the equatorial sphere
    const double radiusSq = sphere.radius * sphere.radius;
    const double distanceSq = (point - sphere.center).squaredNorm();
    return distanceSq < radiusSq - tolerance;
}

Point3D GeometryUtilities3D::computeTriangleCentroid(const Point3D& v1, const Point3D& v2, const Point3D& v3)
{
    return Point3D((v1.x() + v2.x() + v3.x()) / 3.0,
                   (v1.y() + v2.y() + v3.y()) / 3.0,
                   (v1.z() + v2.z() + v3.z()) / 3.0);
}

double GeometryUtilities3D::computePointToTriangleCentroidDistance(const Point3D& point,
                                                                   const Point3D& v1,
                                                                   const Point3D& v2,
                                                                   const Point3D& v3)
{
    Point3D centroid = computeTriangleCentroid(v1, v2, v3);
    return computeEdgeLength(point, centroid);
}

} // namespace Meshing
