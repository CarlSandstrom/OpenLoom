#include "ConstrainedDelaunay2D.h"
#include "Delaunay2D.h"
#include "MeshDebugUtils2D.h"
#include "MeshOperations2D.h"
#include "Meshing/Core/2D/MeshingContext2D.h"
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

ConstrainedDelaunay2D::ConstrainedDelaunay2D(MeshingContext2D& context,
                                             const DiscretizationResult2D& discretization,
                                             const std::vector<Point2D>& additionalPoints,
                                             const std::string& debugExportFilenamePrefix) :
    context_(&context),
    discretization_(discretization),
    additionalPoints_(additionalPoints),
    debugExportFilenamePrefix_(debugExportFilenamePrefix),
    meshData2D_(&context.getMeshData()),
    meshOperations_(&context.getOperations())
{
}

void ConstrainedDelaunay2D::triangulate()
{
    // Add additional points to the discretized points
    std::vector<Point2D> allPoints = discretization_.points;
    std::vector<std::vector<std::string>> allGeometryIds = discretization_.geometryIds;

    allPoints.insert(allPoints.end(), additionalPoints_.begin(), additionalPoints_.end());
    allGeometryIds.insert(allGeometryIds.end(), additionalPoints_.size(), std::vector<std::string>{});

    // Create Delaunay triangulation
    Delaunay2D delaunay(allPoints, meshData2D_, allGeometryIds);
    delaunay.triangulate();
    pointIndexToNodeIdMap_ = delaunay.getPointIndexToNodeIdMap();

    // Extract constrained edges and store in MeshData2D
    auto curveSegmentManager = meshOperations_->getQueries().extractConstrainedEdges(
        context_->getTopology(),
        discretization_.cornerIdToPointIndexMap,
        delaunay.getPointIndexToNodeIdMap(),
        discretization_.edgeIdToPointIndicesMap,
        discretization_.tParameters,
        discretization_.geometryIds);

    meshOperations_->getMutator().setCurveSegmentManager(std::move(curveSegmentManager));

    exportAndVerifyMesh(*meshData2D_, debugExportFilenamePrefix_, exportCounter_);

    // Enforce all constrained edges
    bool allConstrainedEdgesPresent = false;
    while (!allConstrainedEdgesPresent)
    {
        allConstrainedEdgesPresent = true;
        for (const auto& [segId, segment] : meshData2D_->getCurveSegmentManager().getAllSegments())
        {
            allConstrainedEdgesPresent = allConstrainedEdgesPresent &&
                                         meshOperations_->enforceEdge(segment.nodeId1, segment.nodeId2);
        }
    }
    exportAndVerifyMesh(*meshData2D_, debugExportFilenamePrefix_, exportCounter_);

    // Classify triangles as interior/exterior using flood fill algorithm
    // This approach uses mesh topology (constraint edges) instead of geometry queries,
    // making it robust regardless of mesh coarseness relative to geometry features
    meshOperations_->classifyAndRemoveExteriorTriangles();
    exportAndVerifyMesh(*meshData2D_, debugExportFilenamePrefix_, exportCounter_);
}

} // namespace Meshing
