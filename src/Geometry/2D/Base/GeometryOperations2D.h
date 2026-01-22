#pragma once

#include "Common/Types.h"
#include "DiscretizationSettings2D.h"
#include "GeometryCollection2D.h"
#include <Eigen/Core>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace Topology2D
{
class Topology2D;
}

namespace Geometry2D
{

class IFace2D;

/**
 * @brief Utility operations for 2D geometry collections
 *
 * Provides helper methods for extracting and processing geometry data,
 * such as converting corners to point arrays with index mappings.
 */
class GeometryOperations2D
{
public:
    struct PointExtractionResult
    {
        std::vector<Meshing::Point2D> points;                               ///< Extracted point array
        std::vector<std::vector<double>> tParameters;                       ///< Edge parameter t values for each point (empty vector for interior points, one entry per edge for boundary points)
        std::vector<std::vector<std::string>> geometryIds;                  ///< Geometry IDs for each point (empty vector for interior points, corresponding edge IDs for boundary points)
        std::map<std::string, size_t> cornerIdToPointIndexMap;              ///< Maps corner IDs to indices in points vector
        std::map<std::string, std::vector<size_t>> edgeIdToPointIndicesMap; ///< Maps edge IDs to ordered list of point indices along the edge (including start and end corners)
    };

    explicit GeometryOperations2D(const GeometryCollection2D& geometry);

    PointExtractionResult extractCornerPoints() const;

    PointExtractionResult extractPointsWithEdgeDiscretization(const Topology2D::Topology2D& topology,
                                                              const DiscretizationSettings2D& settings = DiscretizationSettings2D()) const;

    Eigen::Vector2d computeEdgeTangentAtParameter(const IEdge2D& edge,
                                                  double parameter) const;

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
