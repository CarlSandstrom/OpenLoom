#include "Meshing/Core/3D/RCDT/RCDTContext.h"

#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Meshing/Core/3D/General/BoundaryDiscretizer3D.h"
#include "Meshing/Core/3D/General/DiscretizationResult3D.h"
#include "Meshing/Core/3D/General/MeshingContext3D.h"
#include "Meshing/Core/3D/RCDT/CurveSegmentOperations.h"
#include "Meshing/Core/3D/RCDT/RCDTRefiner.h"
#include "Meshing/Core/3D/RCDT/RestrictedTriangulation.h"
#include "Meshing/Core/3D/Volume/Delaunay3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Meshing/Data/Base/MeshConnectivity.h"
#include "Meshing/Data/CurveSegmentManager.h"
#include "Topology/Topology3D.h"
#include "spdlog/spdlog.h"

#include <algorithm>
#include <unordered_map>

namespace Meshing
{

RCDTContext::RCDTContext(const Geometry3D::GeometryCollection3D& geometry,
                         const Topology3D::Topology3D& topology,
                         const Geometry3D::DiscretizationSettings3D& discretizationSettings,
                         const RCDTQualitySettings& qualitySettings) :
    geometry_(&geometry),
    topology_(&topology),
    discretizationSettings_(discretizationSettings),
    qualitySettings_(qualitySettings)
{
}

RCDTContext::~RCDTContext() = default;
RCDTContext::RCDTContext(RCDTContext&&) noexcept = default;
RCDTContext& RCDTContext::operator=(RCDTContext&&) noexcept = default;

void RCDTContext::buildInitial()
{
    spdlog::info("RCDTContext::buildInitial: discretizing boundary (edges + corners only)");

    // Phase 1.1: Discretize edges and corners only — no surface interior grid.
    const Geometry3D::DiscretizationSettings3D edgeOnlySettings(
        discretizationSettings_.getNumSegmentsPerEdge(),
        discretizationSettings_.getMaxAngleBetweenSegments(),
        0);

    BoundaryDiscretizer3D discretizer(*geometry_, *topology_, edgeOnlySettings);
    discretizer.discretize();
    auto discretizationResult = discretizer.releaseDiscretizationResult();

    spdlog::info("RCDTContext::buildInitial: {} points after discretization",
                 discretizationResult->points.size());

    // Phase 1.2: Enrich edge-point geometry IDs with adjacent surface IDs.
    // This ensures RestrictedTriangulation::classifyFace sees explicit surface
    // membership for nodes that lie on topology edges.
    std::unordered_map<std::string, std::vector<std::string>> edgeToAdjacentSurfaces;
    for (const auto& edgeId : topology_->getAllEdgeIds())
        edgeToAdjacentSurfaces[edgeId] = topology_->getEdge(edgeId).getAdjacentSurfaceIds();

    auto enrichedGeometryIds = discretizationResult->geometryIds;
    for (auto& ids : enrichedGeometryIds)
    {
        std::vector<std::string> toAdd;
        for (const auto& geometryId : ids)
        {
            auto it = edgeToAdjacentSurfaces.find(geometryId);
            if (it == edgeToAdjacentSurfaces.end())
                continue;
            for (const auto& surfaceId : it->second)
            {
                if (std::find(ids.begin(), ids.end(), surfaceId) == ids.end() &&
                    std::find(toAdd.begin(), toAdd.end(), surfaceId) == toAdd.end())
                {
                    toAdd.push_back(surfaceId);
                }
            }
        }
        ids.insert(ids.end(), toAdd.begin(), toAdd.end());
    }

    // Phase 1.3: Build the initial 3D Delaunay triangulation.
    meshingContext_ = std::make_unique<MeshingContext3D>(*geometry_, *topology_);
    auto& meshData = meshingContext_->getMeshData();

    Delaunay3D delaunay(discretizationResult->points, &meshData, enrichedGeometryIds);
    delaunay.triangulate();
    const auto pointIndexToNodeIdMap = delaunay.getPointIndexToNodeIdMap();

    spdlog::info("RCDTContext::buildInitial: Delaunay3D produced {} nodes, {} elements",
                 meshData.getNodeCount(), meshData.getElementCount());

    // Phase 1.4: Rebuild connectivity.
    meshingContext_->rebuildConnectivity();

    // Phase 1.5: Build RestrictedTriangulation.
    restrictedTriangulation_ = std::make_unique<RestrictedTriangulation>();
    const MeshConnectivity connectivity(meshData);
    restrictedTriangulation_->buildFrom(meshData, connectivity, *geometry_, *topology_);

    spdlog::info("RCDTContext::buildInitial: {} restricted faces",
                 restrictedTriangulation_->getRestrictedFaces().size());

    // Phase 1.6: Build cornerIdToNodeId from discretization result + Delaunay mapping.
    std::unordered_map<std::string, size_t> cornerIdToNodeId;
    for (const auto& [cornerId, pointIndex] : discretizationResult->cornerIdToPointIndexMap)
    {
        auto nodeIt = pointIndexToNodeIdMap.find(pointIndex);
        if (nodeIt != pointIndexToNodeIdMap.end())
            cornerIdToNodeId[cornerId] = nodeIt->second;
    }

    // Phase 1.7: Build curve segments. buildCurveSegments requires a mutable
    // CurveSegmentManager, so we build into a temporary and transfer to MeshData3D
    // via the mutator.
    CurveSegmentManager temporarySegmentManager;
    buildCurveSegments(temporarySegmentManager, *topology_, *geometry_, cornerIdToNodeId);

    auto& mutator = meshingContext_->getMutator();
    for (const auto& [segmentId, segment] : temporarySegmentManager.getAllSegments())
        mutator.addCurveSegment(segment);

    spdlog::info("RCDTContext::buildInitial: {} curve segments added",
                 meshData.getCurveSegmentManager().size());
}

void RCDTContext::refine()
{
    spdlog::info("RCDTContext::refine: starting RCDT refinement");
    RCDTRefiner refiner(*meshingContext_, *restrictedTriangulation_, qualitySettings_);
    refiner.refine();
    spdlog::info("RCDTContext::refine: done");
}

SurfaceMesh3D RCDTContext::buildSurfaceMesh() const
{
    SurfaceMesh3D mesh;
    const auto& meshData = meshingContext_->getMeshData();

    // Nodes: size the vector to max node ID + 1 (index in vector == node ID).
    if (meshData.getNodeCount() > 0)
    {
        size_t maxNodeId = 0;
        for (const auto& [nodeId, node] : meshData.getNodes())
            maxNodeId = std::max(maxNodeId, nodeId);

        mesh.nodes.resize(maxNodeId + 1);
        for (const auto& [nodeId, node] : meshData.getNodes())
            mesh.nodes[nodeId] = node->getCoordinates();
    }

    // Triangles and per-face groups: iterate restricted faces grouped by surface ID.
    for (const auto& [faceKey, surfaceId] : restrictedTriangulation_->getRestrictedFaces())
    {
        const size_t triangleIndex = mesh.triangles.size();
        mesh.triangles.push_back({faceKey.nodeIds[0], faceKey.nodeIds[1], faceKey.nodeIds[2]});
        mesh.faceTriangleIds[surfaceId].push_back(triangleIndex);
    }

    // Edge node sequences: reconstruct ordered node lists from curve segments.
    const auto& curveSegmentManager = meshData.getCurveSegmentManager();
    for (const auto& edgeId : topology_->getAllEdgeIds())
    {
        const auto segments = curveSegmentManager.getSegmentsForEdge(edgeId);
        if (segments.empty())
            continue;

        std::vector<size_t> nodeIds;
        nodeIds.push_back(segments[0].nodeId1);
        for (const auto& segment : segments)
            nodeIds.push_back(segment.nodeId2);

        mesh.edgeNodeIds[edgeId] = std::move(nodeIds);
    }

    spdlog::debug("RCDTContext::buildSurfaceMesh: {} nodes, {} triangles, {} faces, {} edges",
                  mesh.nodes.size(), mesh.triangles.size(),
                  mesh.faceTriangleIds.size(), mesh.edgeNodeIds.size());

    return mesh;
}

} // namespace Meshing
