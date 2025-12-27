#pragma once

#include "Common/Types.h"
#include "DiscretizationSettings2D.h"
#include "GeometryCollection2D.h"
#include <map>
#include <string>
#include <vector>

namespace Topology2D
{
class Topology2D;
}

namespace Geometry2D
{

/**
 * @brief Utility operations for 2D geometry collections
 *
 * Provides helper methods for extracting and processing geometry data,
 * such as converting corners to point arrays with index mappings.
 */
class GeometryOperations2D
{
public:
    /**
     * @brief Result of point extraction operations
     */
    struct PointExtractionResult
    {
        std::vector<Meshing::Point2D> points;                     ///< Extracted point array
        std::map<std::string, size_t> cornerIdToPointIndexMap;    ///< Maps corner IDs to indices in points vector
    };

    /**
     * @brief Construct operations for a specific geometry collection
     * @param geometry The geometry collection to operate on
     */
    explicit GeometryOperations2D(const GeometryCollection2D& geometry);

    /**
     * @brief Extract corner points from geometry collection
     *
     * Converts all corners in the geometry to a point vector while
     * maintaining a mapping from corner IDs to point indices.
     *
     * @return Points vector and corner-to-index mapping
     */
    PointExtractionResult extractCornerPoints() const;

    /**
     * @brief Extract corner points with edge discretization
     *
     * Extracts corner points and adds intermediate points along edges
     * according to the discretization settings.
     *
     * @param topology The topology defining edges
     * @param settings Discretization settings controlling edge subdivision
     * @return Points vector and corner-to-index mapping (intermediate points not in map)
     */
    PointExtractionResult extractPointsWithEdgeDiscretization(
        const Topology2D::Topology2D& topology,
        const DiscretizationSettings2D& settings = DiscretizationSettings2D()) const;

private:
    const GeometryCollection2D& geometry_;  ///< Reference to the geometry collection
};

} // namespace Geometry2D
