#pragma once

#include "Common/Types.h"
#include "DiscretizationResult2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace Meshing
{

class MeshingContext2D;
class MeshOperations2D;

class ConstrainedDelaunay2D
{
public:
    /**
     * @brief Build a constrained Delaunay triangulation into a MeshingContext2D
     *
     * @param context The 2D meshing context containing geometry and topology
     * @param discretization Pre-computed edge discretization from EdgeDiscretizer2D
     * @param additionalPoints Additional points to include in triangulation (optional)
     * @param debugExportFilenamePrefix Prefix for debug export filenames (optional)
     * @return Map from input point index to the node id it became
     */
    static std::map<size_t, size_t> triangulate(
        MeshingContext2D& context,
        const DiscretizationResult2D& discretization,
        const std::vector<Point2D>& additionalPoints = {},
        const std::string& debugExportFilenamePrefix = "constrained_delaunay");
};


} // namespace Meshing
