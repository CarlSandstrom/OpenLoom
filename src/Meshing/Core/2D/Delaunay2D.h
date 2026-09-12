#pragma once

#include "Common/Types.h"
#include "Meshing/Data/2D/MeshData2D.h"
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
    /// Triangulates points into meshData.
    /// geometryIds provides edge IDs for boundary points (empty for interior points).
    /// Returns the map from input point index to the node id it became.
    static std::map<size_t, size_t> triangulate(const std::vector<Point2D>& points,
                                                MeshData2D& meshData,
                                                const std::vector<std::vector<std::string>>& geometryIds = {});
};

} // namespace Meshing
