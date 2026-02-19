#pragma once

#include "Common/Types.h"
#include <map>
#include <string>
#include <vector>

namespace Meshing
{

class MeshData3D;

/**
 * @brief Simple Delaunay tetrahedralization in 3D
 *
 * This class provides a wrapper around the Bowyer-Watson algorithm for
 * creating a Delaunay tetrahedralization from a set of 3D points.
 * Mirrors the API of Delaunay2D for consistency.
 */
class Delaunay3D
{
public:
    /**
     * @brief Construct a Delaunay3D triangulator with input points
     * @param points Vector of Point3D representing the input vertices
     * @param meshData Pointer to mesh data storage
     * @param edgeParameters Edge parameters for each point (empty vector for interior points)
     * @param geometryIds Geometry IDs for each point (corner/edge/surface IDs)
     */
    explicit Delaunay3D(const std::vector<Point3D>& points,
                        MeshData3D* meshData,
                        const std::vector<std::vector<double>>& edgeParameters = {},
                        const std::vector<std::vector<std::string>>& geometryIds = {});

    /**
     * @brief Perform the Delaunay tetrahedralization
     */
    void triangulate();

    /**
     * @brief Get mapping from input point index to mesh node ID
     * @return Map from point index to node ID
     */
    std::map<size_t, size_t> getPointIndexToNodeIdMap() const { return pointIndexToNodeIdMap_; }

private:
    MeshData3D* meshData_;
    std::vector<Point3D> points_;
    std::vector<std::vector<double>> edgeParameters_;
    std::vector<std::vector<std::string>> geometryIds_;
    std::map<size_t, size_t> pointIndexToNodeIdMap_;
};

} // namespace Meshing
