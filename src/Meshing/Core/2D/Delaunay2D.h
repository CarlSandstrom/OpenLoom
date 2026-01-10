#pragma once

#include "Common/Types.h"
#include "Meshing/Core/2D/MeshOperations2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/MeshMutator2D.h"
#include <array>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace Meshing
{

/**
 * @brief Simple Delaunay triangulation in 2D
 *
 * This class provides a basic framework for implementing Delaunay triangulation
 * algorithms. It takes a vector of Point2D and produces triangles.
 */
class Delaunay2D
{
public:
    /**
     * @brief Construct a Delaunay2D triangulator with input points
     * @param points Vector of Point2D representing the input vertices
     * @param meshData Pointer to mesh data storage
     * @param tParameters Edge parameters for each point (empty vector for interior points, one or more entries for boundary points)
     * @param geometryIds Geometry IDs for each point (empty vector for interior points, corresponding edge IDs for boundary points)
     */
    explicit Delaunay2D(const std::vector<Point2D>& points,
                        MeshData2D* meshData,
                        const std::vector<std::vector<double>>& tParameters = {},
                        const std::vector<std::vector<std::string>>& geometryIds = {});

    /**
     * @brief Perform the Delaunay triangulation
     */
    void triangulate();

    std::map<size_t, size_t> getPointIndexToNodeIdMap() const { return pointIndexToNodeIdMap_; }

private:
    MeshData2D* meshData_;
    MeshMutator2D meshMutator_;
    MeshOperations2D meshOperations_;
    std::vector<Point2D> points_;
    std::vector<std::vector<double>> tParameters_;
    std::vector<std::vector<std::string>> geometryIds_;
    std::map<size_t, size_t> pointIndexToNodeIdMap_;
};

} // namespace Meshing
