#pragma once

#include "Common/Types.h"
#include "Meshing/Core/3D/General/GeometryStructures3D.h"

namespace Meshing
{

/// Static utilities for pure 3D geometric computations that don't require mesh context.
class GeometryUtilities3D
{
public:
    /// Tests if a point is inside a circumscribed sphere.
    static bool isPointInsideCircumscribingSphere(const CircumscribedSphere& sphere,
                                                  const Point3DRef point,
                                                  double tolerance = 1e-12);

    /// Computes the Euclidean distance between two points.
    static double computeEdgeLength(const Point3D& p1, const Point3D& p2);

    /// Creates a diametral sphere with the given segment as diameter.
    /// The center is the midpoint and radius is half the segment length.
    static DiametralSphere createDiametralSphere(const Point3D& p1, const Point3D& p2);

    /// Tests if a point is inside the diametral sphere (encroaches the subsegment).
    /// Excludes the segment endpoints themselves.
    static bool isPointInDiametralSphere(const DiametralSphere& sphere,
                                         const Point3D& point,
                                         double tolerance = 1e-12);

    /// Creates an equatorial sphere passing through three points of a triangle.
    /// The center lies in the plane of the triangle.
    static EquatorialSphere createEquatorialSphere(const Point3D& p1,
                                                   const Point3D& p2,
                                                   const Point3D& p3);

    /// Tests if a point is inside the equatorial sphere (encroaches the subfacet).
    /// Point must be non-coplanar with the triangle for encroachment.
    static bool isPointInEquatorialSphere(const EquatorialSphere& sphere,
                                          const Point3D& point,
                                          const Point3D& triangleP1,
                                          const Point3D& triangleP2,
                                          const Point3D& triangleP3,
                                          double tolerance = 1e-12);

    /// Computes the centroid of a triangle given its three vertices.
    static Point3D computeTriangleCentroid(const Point3D& v1, const Point3D& v2, const Point3D& v3);

    /// Computes the distance from a point to the centroid of a triangle.
    /// This is an approximate distance to triangle, useful for flood-fill seeding.
    static double computePointToTriangleCentroidDistance(const Point3D& point,
                                                         const Point3D& v1,
                                                         const Point3D& v2,
                                                         const Point3D& v3);
};

} // namespace Meshing
