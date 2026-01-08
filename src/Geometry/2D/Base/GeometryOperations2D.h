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
        std::vector<std::optional<double>> tParameters;                     ///< Edge parameter t for each point (nullopt for corners)
        std::vector<std::string> geometryIds;                               ///< Geometry ID for each point (empty for corners, edge ID for edge points)
        std::map<std::string, size_t> cornerIdToPointIndexMap;              ///< Maps corner IDs to indices in points vector
        std::map<std::string, std::vector<size_t>> edgeIdToPointIndicesMap; ///< Maps edge IDs to ordered list of point indices along the edge (including start and end corners)
    };

    explicit GeometryOperations2D(const GeometryCollection2D& geometry);

    PointExtractionResult extractCornerPoints() const;

    PointExtractionResult extractPointsWithEdgeDiscretization(const Topology2D::Topology2D& topology,
                                                              const DiscretizationSettings2D& settings = DiscretizationSettings2D()) const;

    Eigen::Vector2d computeEdgeTangentAtParameter(const IEdge2D& edge,
                                                  double parameter) const;

private:
    const GeometryCollection2D& geometry_;
};

} // namespace Geometry2D
