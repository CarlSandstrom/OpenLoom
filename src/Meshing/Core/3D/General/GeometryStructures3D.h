#pragma once

#include "Common/Types.h"
#include <string>

namespace Meshing
{

struct CircumscribedSphere
{
    Point3D center;
    double radius;
};

/**
 * @brief Represents a constrained triangular face in 3D mesh
 *
 * A subfacet is part of a constrained facet (surface) that must be
 * preserved in the final mesh. During refinement, facets may be
 * subdivided into multiple subfacets.
 */
struct ConstrainedSubfacet3D
{
    size_t nodeId1;         // First vertex node ID
    size_t nodeId2;         // Second vertex node ID
    size_t nodeId3;         // Third vertex node ID
    std::string geometryId; // Parent surface geometry ID
};

/**
 * @brief Diametral sphere of a line segment
 *
 * The diametral sphere is the smallest sphere that encloses a segment.
 * Its center is the midpoint of the segment and radius is half the length.
 * Used for subsegment encroachment tests.
 */
struct DiametralSphere
{
    Point3D center;
    double radius;
};

/**
 * @brief Equatorial sphere of a triangle
 *
 * The equatorial sphere is the smallest sphere passing through the three
 * vertices of a triangle. Its center lies in the plane of the triangle.
 * Used for subfacet encroachment tests.
 */
struct EquatorialSphere
{
    Point3D center;
    double radius;
};

} // namespace Meshing