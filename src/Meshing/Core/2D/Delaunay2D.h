#pragma once

#include "Common/Types.h"
#include "Computer2D.h"
#include "Meshing/Core/2D/MeshOperations2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/MeshMutator2D.h"
#include <array>
#include <map>
#include <memory>
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
     */
    explicit Delaunay2D(const std::vector<Point2D>& points, MeshData2D* meshData);

    /**
     * @brief Perform the Delaunay triangulation
     * @return Vector of triangles, where each triangle is an array of 3 indices into the input points
     */
    void triangulate();

    std::map<size_t, size_t> getPointIndexToNodeIdMap() const { return pointIndexToNodeIdMap_; }

private:
    MeshData2D* meshData_;
    MeshMutator2D meshMutator_;
    MeshOperations2D meshOperations_;
    Computer2D computer_;
    std::vector<Point2D> points_;
    std::map<size_t, size_t> pointIndexToNodeIdMap_;
};

} // namespace Meshing
