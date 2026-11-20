#pragma once

#include "Common/Types.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include <map>
#include <vector>

namespace Meshing
{

class Delaunay2D
{
public:
    /// Triangulates points into meshData.
    /// Returns the map from input point index to the node id it became.
    static std::map<size_t, NodeId> triangulate(const std::vector<Point2D>& points, MeshData2D& meshData);
};

} // namespace Meshing
