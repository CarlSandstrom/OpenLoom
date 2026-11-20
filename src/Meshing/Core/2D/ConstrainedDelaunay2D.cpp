#include "Meshing/Core/2D/ConstrainedDelaunay2D.h"
#include "Common/Exceptions/MeshException.h"
#include "Meshing/Core/2D/Delaunay2D.h"
#include "Meshing/Core/2D/General/MeshDebugUtilities2D.h"
#include "Meshing/Core/2D/General/MeshOperations2D.h"
#include "Meshing/Core/2D/General/MeshQueries2D.h"
#include "Meshing/Core/2D/General/MeshingContext2D.h"
#include "Meshing/Data/2D/DiscretizationResult2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/MeshMutator2D.h"
#include <algorithm>
#include <map>
#include <string>
#include <vector>

namespace Meshing
{

namespace
{

size_t enforceConstrainedEdges(const MeshData2D& meshData, MeshOperations2D& meshOperations);

} // namespace

std::map<size_t, NodeId> ConstrainedDelaunay2D::triangulate(
    MeshingContext2D& context,
    const DiscretizationResult2D& discretization,
    const std::vector<Point2D>& additionalPoints,
    const std::string& debugExportFilenamePrefix)
{
    MeshData2D& meshData2D = context.getMeshData();
    MeshOperations2D& meshOperations = context.getOperations();
    size_t exportCounter = 0;

    std::vector<Point2D> allPoints = discretization.points;
    std::vector<std::vector<std::string>> allGeometryIds = discretization.geometryIds;

    allPoints.insert(allPoints.end(), additionalPoints.begin(), additionalPoints.end());
    allGeometryIds.insert(allGeometryIds.end(), additionalPoints.size(), std::vector<std::string>{});

    std::map<size_t, NodeId> pointIndexToNodeIdMap =
        Delaunay2D::triangulate(allPoints, meshData2D, allGeometryIds);

    auto curveSegmentManager = meshOperations.getQueries().extractConstrainedEdges(
        context.getTopology(),
        discretization.cornerIdToPointIndexMap,
        pointIndexToNodeIdMap,
        discretization.edgeIdToPointIndicesMap,
        discretization.tParameters,
        discretization.geometryIds);

    meshOperations.getMutator().setCurveSegmentManager(std::move(curveSegmentManager));

    exportAndVerifyMesh(meshData2D, debugExportFilenamePrefix, exportCounter);

    // Enforcing one edge can retriangulate around another, so passes repeat until
    // every edge is present. A pass that recovers nothing would repeat forever.
    size_t missingEdgeCount = enforceConstrainedEdges(meshData2D, meshOperations);
    while (missingEdgeCount > 0)
    {
        size_t previousMissingEdgeCount = missingEdgeCount;
        missingEdgeCount = enforceConstrainedEdges(meshData2D, meshOperations);
        if (missingEdgeCount >= previousMissingEdgeCount)
        {
            OPENLOOM_THROW_CODE(OpenLoom::MeshException,
                                OpenLoom::MeshException::ErrorCode::CONSTRAINT_VIOLATION,
                                "ConstrainedDelaunay2D: " + std::to_string(missingEdgeCount) +
                                    " constrained edges could not be enforced");
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

namespace
{

// Returns how many constrained edges are still missing after the pass.
size_t enforceConstrainedEdges(const MeshData2D& meshData, MeshOperations2D& meshOperations)
{
    auto& segments = meshData.getCurveSegmentManager().getAllSegments();
    for (const auto& [segmentId, segment] : segments)
        meshOperations.enforceEdge(segment.nodeId1, segment.nodeId2);

    return std::count_if(segments.begin(), segments.end(),
                         [&meshOperations](const auto& entry)
                         {
                             auto& segment = entry.second;
                             return meshOperations.getQueries()
                                 .findTrianglesAdjacentToEdge(segment.nodeId1, segment.nodeId2)
                                 .empty();
                         });
}

} // namespace

} // namespace Meshing
