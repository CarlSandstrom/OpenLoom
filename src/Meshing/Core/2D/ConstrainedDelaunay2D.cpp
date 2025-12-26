#include "ConstrainedDelaunay2D.h"
#include "Delaunay2D.h"
#include "Geometry2D/Corner2D.h"
#include "Geometry2D/GeometryCollection2D.h"
#include "Geometry2D/IEdge2D.h"
#include "MeshOperations2D.h"
#include "Meshing/Core/2D/MeshVerifier.h"
#include "Meshing/Core/2D/MeshingContext2D.h"
#include "Meshing/Data/MeshMutator2D.h"
#include "Topology2D/Corner2D.h"
#include "Topology2D/Edge2D.h"
#include "Topology2D/Topology2D.h"
#include "spdlog/spdlog.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>

#include "Export/VtkExporter.h"
#include "Utils/MeshLogger.h"

namespace Meshing
{

ConstrainedDelaunay2D::ConstrainedDelaunay2D(MeshingContext2D& context, const std::vector<Point2D>& additionalPoints) :
    context_(&context),
    meshData2D_(&context.getMeshData()),
    meshMutator_(&context.getMutator()),
    meshOperations_(&context.getOperations())
{
    // Extract corner points from geometry and generate a vector of points along with a mapping from cornerId to point index
    std::vector<Point2D> points;
    std::map<std::string, size_t> cornerIdToPointIndexMap;
    for (const auto& id : context_->getGeometry().getAllCornerIds())
    {
        const auto* corner = context_->getGeometry().getCorner(id);
        points.push_back(corner->getPoint());
        cornerIdToPointIndexMap[id] = points.size() - 1;
    }

    // Refine edges to add intermediate points along constrained edges. This is just temporary and will be replaced with proper edge sampling later.
    for (const auto& edgeId : context_->getTopology().getAllEdgeIds())
    {
        const auto* edgeGeometryPtr = context_->getGeometry().getEdge(edgeId);
        auto parameterBounds = edgeGeometryPtr->getParameterBounds();
        size_t numSegments = 1; // Fixed number of segments for now
        for (size_t i = 1; i < numSegments; ++i)
        {
            double t = parameterBounds.first + i * (parameterBounds.second - parameterBounds.first) / numSegments;
            Point2D point = edgeGeometryPtr->getPoint(t);
            points.push_back(point);
        }
    }

    for (const auto& additionalPoint : additionalPoints)
    {
        points.push_back(additionalPoint);
    }

    // Create Delaunay triangulator with the extracted points
    Delaunay2D delaunay(points, meshData2D_);
    delaunay.triangulate();

    // The nodes in the mesh is in the same order as the input points, so we can find which node is which corner.
    auto pointIndexToNodeIdMap = delaunay.getPointIndexToNodeIdMap();

    std::vector<std::pair<size_t, size_t>> constrainedEdges;

    for (auto edge : context_->getTopology().getAllEdgeIds())
    {
        const auto* edgeGeometryPtr = context_->getGeometry().getEdge(edge);
        const auto edgeTopologyPtr = context_->getTopology().getEdge(edge);

        std::pair<size_t, size_t> edgeNodeIds = {
            pointIndexToNodeIdMap[cornerIdToPointIndexMap[edgeTopologyPtr.getStartCornerId()]],
            pointIndexToNodeIdMap[cornerIdToPointIndexMap[edgeTopologyPtr.getEndCornerId()]]};
        constrainedEdges.push_back(edgeNodeIds);
        spdlog::info("Edge {}: Node IDs ({}, {})", edge, edgeNodeIds.first, edgeNodeIds.second);
    }

    Export::VtkExporter exporter;
    MeshData3D meshData3D(*meshData2D_);
    size_t counter = 0;
    exporter.exportMesh(meshData3D, "constrained_delaunay_" + std::to_string(counter++) + ".vtu");

    bool allConstrinedEdgesPresent = false;
    MeshLogger::logMeshData2D(*meshData2D_);
    MeshVerifier verifier(*meshData2D_);

    auto result = verifier.verify();
    if (!result.isValid)
    {
        for (const auto& error : result.errors)
        {
            spdlog::error(" - {}", error);
        }
        throw std::runtime_error("Mesh verification failed");
    }

    while (!allConstrinedEdgesPresent)
    {
        allConstrinedEdgesPresent = true;
        for (const auto& edge : constrainedEdges)
        {
            allConstrinedEdgesPresent = allConstrinedEdgesPresent && meshOperations_->enforceEdge(edge.first, edge.second);
            MeshLogger::logMeshData2D(*meshData2D_);
            MeshData3D meshData3D(*meshData2D_);
            exporter.exportMesh(meshData3D, "constrained_delaunay_" + std::to_string(counter++) + ".vtu");

            MeshVerifier verifier(*meshData2D_);
            auto result = verifier.verify();
            if (!result.isValid)
            {
                spdlog::error("Mesh invalid after enforcing edge ({}, {})", edge.first, edge.second);
                for (const auto& error : result.errors)
                {
                    spdlog::error(" - {}", error);
                }
                throw std::runtime_error("Mesh verification failed");
            }
        }
    }
}

ConstrainedDelaunay2D::~ConstrainedDelaunay2D() {

};

void ConstrainedDelaunay2D::triangulate()
{
}

} // namespace Meshing
