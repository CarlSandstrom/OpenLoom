#include "ConstrainedDelaunay2D.h"
#include "Delaunay2D.h"
#include "MeshDebugUtils2D.h"
#include "MeshOperations2D.h"
#include "Meshing/Core/2D/MeshingContext2D.h"
#include "Meshing/Data/2D/MeshMutator2D.h"
#include "Utils/MeshLogger.h"
#include "spdlog/spdlog.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <queue>

namespace Meshing
{

std::map<size_t, size_t> ConstrainedDelaunay2D::triangulate(
    MeshingContext2D& context,
    const DiscretizationResult2D& discretization,
    const std::vector<Point2D>& additionalPoints,
    const std::string& debugExportFilenamePrefix)
{
    MeshData2D& meshData2D = context.getMeshData();
    MeshOperations2D& meshOperations = context.getOperations();
    size_t exportCounter = 0;

    // Add additional points to the discretized points
    std::vector<Point2D> allPoints = discretization.points;
    std::vector<std::vector<std::string>> allGeometryIds = discretization.geometryIds;

    allPoints.insert(allPoints.end(), additionalPoints.begin(), additionalPoints.end());
    allGeometryIds.insert(allGeometryIds.end(), additionalPoints.size(), std::vector<std::string>{});

    // Create Delaunay triangulation
    const std::map<size_t, size_t> pointIndexToNodeIdMap =
        Delaunay2D::triangulate(allPoints, meshData2D, allGeometryIds);

    // Extract constrained edges and store in MeshData2D
    auto curveSegmentManager = meshOperations.getQueries().extractConstrainedEdges(
        context.getTopology(),
        discretization.cornerIdToPointIndexMap,
        pointIndexToNodeIdMap,
        discretization.edgeIdToPointIndicesMap,
        discretization.tParameters,
        discretization.geometryIds);

    meshOperations.getMutator().setCurveSegmentManager(std::move(curveSegmentManager));

    exportAndVerifyMesh(meshData2D, debugExportFilenamePrefix, exportCounter);

    // Enforce all constrained edges
    bool allConstrainedEdgesPresent = false;
    while (!allConstrainedEdgesPresent)
    {
        allConstrainedEdgesPresent = true;
        for (const auto& [segId, segment] : meshData2D.getCurveSegmentManager().getAllSegments())
        {
            allConstrainedEdgesPresent = allConstrainedEdgesPresent &&
                                         meshOperations.enforceEdge(segment.nodeId1, segment.nodeId2);
        }
    }
    exportAndVerifyMesh(meshData2D, debugExportFilenamePrefix, exportCounter);

    // Classify triangles as interior/exterior using flood fill algorithm
    // This approach uses mesh topology (constraint edges) instead of geometry queries,
    // making it robust regardless of mesh coarseness relative to geometry features
    meshOperations.classifyAndRemoveExteriorTriangles();
    exportAndVerifyMesh(meshData2D, debugExportFilenamePrefix, exportCounter);

    return pointIndexToNodeIdMap;
}

} // namespace Meshing
