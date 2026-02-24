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
                                             const std::vector<Point2D>& additionalPoints) :
    context_(&context),
    discretization_(discretization),
    additionalPoints_(additionalPoints),
    meshData2D_(&context.getMeshData()),
    meshOperations_(&context.getOperations())
{
}

void ConstrainedDelaunay2D::triangulate()
{
    // Add additional points to the discretized points
    std::vector<Point2D> allPoints = discretization_.points;
    std::vector<std::vector<double>> allTParameters = discretization_.tParameters;
    std::vector<std::vector<std::string>> allGeometryIds = discretization_.geometryIds;

    allPoints.insert(allPoints.end(), additionalPoints_.begin(), additionalPoints_.end());
    // Additional points don't have edge parameters or geometry IDs (empty vectors)
    allTParameters.insert(allTParameters.end(), additionalPoints_.size(), std::vector<double>{});
    allGeometryIds.insert(allGeometryIds.end(), additionalPoints_.size(), std::vector<std::string>{});

    // Create Delaunay triangulation
    Delaunay2D delaunay(allPoints, meshData2D_, allTParameters, allGeometryIds);
    delaunay.triangulate();

    // Extract constrained edges and store in MeshData2D
    auto constrainedEdges = meshOperations_->getQueries().extractConstrainedEdges(context_->getTopology(),
                                                                                  discretization_.cornerIdToPointIndexMap,
                                                                                  delaunay.getPointIndexToNodeIdMap(),
                                                                                  discretization_.edgeIdToPointIndicesMap);

    for (const auto& segment : constrainedEdges)
    {
        meshOperations_->getMutator().addConstrainedSegment(segment);
    }

    exportAndVerifyMesh(*meshData2D_, "constrained_delaunay", exportCounter_);

    // Enforce all constrained edges
    bool allConstrainedEdgesPresent = false;
    while (!allConstrainedEdgesPresent)
    {
        allConstrainedEdgesPresent = true;
        for (const auto& segment : meshData2D_->getConstrainedSegments())
        {
            allConstrainedEdgesPresent = allConstrainedEdgesPresent &&
                                         meshOperations_->enforceEdge(segment.nodeId1, segment.nodeId2);
        }
    }
    exportAndVerifyMesh(*meshData2D_, "constrained_delaunay", exportCounter_);

    // Classify triangles as interior/exterior using flood fill algorithm
    // This approach uses mesh topology (constraint edges) instead of geometry queries,
    // making it robust regardless of mesh coarseness relative to geometry features
    meshOperations_->classifyAndRemoveExteriorTriangles();
    exportAndVerifyMesh(*meshData2D_, "constrained_delaunay", exportCounter_);
}

} // namespace Meshing
