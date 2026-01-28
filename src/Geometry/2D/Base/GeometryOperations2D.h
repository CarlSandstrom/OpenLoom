#pragma once

#include "Common/Types.h"
#include "GeometryCollection2D.h"

namespace Geometry2D
{

class IFace2D;

/**
 * @brief Utility operations for 2D geometry collections
 *
 * Provides helper methods for processing geometry data.
 */
class GeometryOperations2D
{
public:
    explicit GeometryOperations2D(const GeometryCollection2D& geometry);

    /**
     * @brief Check if a point is inside a domain (not in a hole or outside)
     *
     * @param point The point to check
     * @param domainFace The face representing the domain with outer boundary and holes
     * @return true if point is inside the domain or on boundary, false if outside or in hole
     */
    static bool isPointInsideDomain(const Meshing::Point2D& point, const IFace2D& domainFace);

private:
    const GeometryCollection2D& geometry_;
};

} // namespace Geometry2D
