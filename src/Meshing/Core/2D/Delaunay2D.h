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

class Delaunay2D
{
public:
    /// Construct a Delaunay2D triangulator.
    /// geometryIds provides edge IDs for boundary points (empty for interior points).
    explicit Delaunay2D(const std::vector<Point2D>& points,
                        MeshData2D* meshData,
                        const std::vector<std::vector<std::string>>& geometryIds = {});

    void triangulate();

    const std::map<size_t, size_t>& getPointIndexToNodeIdMap() const { return pointIndexToNodeIdMap_; }

private:
    MeshData2D* meshData_;
    MeshMutator2D meshMutator_;
    MeshOperations2D meshOperations_;
    std::vector<Point2D> points_;
    std::vector<std::vector<std::string>> geometryIds_;
    std::map<size_t, size_t> pointIndexToNodeIdMap_;
};

} // namespace Meshing
