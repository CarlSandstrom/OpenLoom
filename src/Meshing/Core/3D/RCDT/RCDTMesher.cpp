#include "Meshing/Core/3D/RCDT/RCDTMesher.h"

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

RCDTMesher::RCDTMesher(const Geometry3D::GeometryCollection3D& geometry,
                       const Topology3D::Topology3D& topology,
                       Geometry3D::DiscretizationSettings3D discretizationSettings,
                       RCDTQualitySettings qualitySettings) :
    context_(geometry, topology, discretizationSettings, qualitySettings)
{
}

RCDTMesher::~RCDTMesher() = default;
RCDTMesher::RCDTMesher(RCDTMesher&&) noexcept = default;
RCDTMesher& RCDTMesher::operator=(RCDTMesher&&) noexcept = default;

SurfaceMesh3D RCDTMesher::mesh()
{
    buildInitial();
    refine();
    return buildSurfaceMesh();
}

void RCDTMesher::buildInitial()
{
    spdlog::info("RCDTMesher::buildInitial: discretizing boundary (edges + corners only)");

    const Geometry3D::DiscretizationSettings3D edgeOnlySettings(
        context_.getDiscretizationSettings().getNumSegmentsPerEdge(),
        context_.getDiscretizationSettings().getMaxAngleBetweenSegments(),
        0);

    BoundaryDiscretizer3D discretizer(context_.getGeometry(), context_.getTopology(), edgeOnlySettings);
    discretizer.discretize();
    auto discretizationResult = discretizer.releaseDiscretizationResult();

    spdlog::info("RCDTMesher::buildInitial: {} points after discretization",
                 discretizationResult->points.size());

    std::unordered_map<std::string, std::vector<std::string>> edgeToAdjacentSurfaces;
    for (const auto& edgeId : context_.getTopology().getAllEdgeIds())
        edgeToAdjacentSurfaces[edgeId] = context_.getTopology().getEdge(edgeId).getAdjacentSurfaceIds();

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

    auto& meshingContext = context_.getMeshingContext();
    auto& meshData = meshingContext.getMeshData();

    Delaunay3D delaunay(discretizationResult->points, &meshData, enrichedGeometryIds);
    delaunay.triangulate();
    const auto pointIndexToNodeIdMap = delaunay.getPointIndexToNodeIdMap();

    spdlog::info("RCDTMesher::buildInitial: Delaunay3D produced {} nodes, {} elements",
                 meshData.getNodeCount(), meshData.getElementCount());

    meshingContext.rebuildConnectivity();

    auto restrictedTriangulation = std::make_unique<RestrictedTriangulation>();
    const MeshConnectivity connectivity(meshData);
    restrictedTriangulation->buildFrom(meshData, connectivity, context_.getGeometry(), context_.getTopology());

    spdlog::info("RCDTMesher::buildInitial: {} restricted faces",
                 restrictedTriangulation->getRestrictedFaces().size());

    CurveSegmentManager temporarySegmentManager;
    buildCurveSegments(temporarySegmentManager, context_.getTopology(), context_.getGeometry(),
                       discretizationResult->edgeIdToPointIndicesMap,
                       pointIndexToNodeIdMap,
                       discretizationResult->edgeParameters);

    auto& mutator = meshingContext.getMutator();
    for (const auto& [segmentId, segment] : temporarySegmentManager.getAllSegments())
        mutator.addCurveSegment(segment);

    spdlog::info("RCDTMesher::buildInitial: {} curve segments added",
                 meshData.getCurveSegmentManager().size());

    context_.setRestrictedTriangulation(std::move(restrictedTriangulation));
}

void RCDTMesher::refine()
{
    spdlog::info("RCDTMesher::refine: starting RCDT refinement");
    RCDTRefiner refiner(context_.getMeshingContext(), context_.getRestrictedTriangulation(),
                        context_.getQualitySettings());
    refiner.refine();
    spdlog::info("RCDTMesher::refine: done");
}

SurfaceMesh3D RCDTMesher::buildSurfaceMesh() const
{
    SurfaceMesh3D surfaceMesh;
    const auto& meshData = context_.getMeshingContext().getMeshData();
    const auto& restrictedTriangulation = context_.getRestrictedTriangulation();

    if (meshData.getNodeCount() > 0)
    {
        size_t maxNodeId = 0;
        for (const auto& [nodeId, node] : meshData.getNodes())
            maxNodeId = std::max(maxNodeId, nodeId);

        surfaceMesh.nodes.resize(maxNodeId + 1);
        for (const auto& [nodeId, node] : meshData.getNodes())
            surfaceMesh.nodes[nodeId] = node->getCoordinates();
    }

    for (const auto& [faceKey, surfaceId] : restrictedTriangulation.getRestrictedFaces())
    {
        const size_t triangleIndex = surfaceMesh.triangles.size();
        surfaceMesh.triangles.push_back({faceKey.nodeIds[0], faceKey.nodeIds[1], faceKey.nodeIds[2]});
        surfaceMesh.faceTriangleIds[surfaceId].push_back(triangleIndex);
    }

    const auto& curveSegmentManager = meshData.getCurveSegmentManager();
    for (const auto& edgeId : context_.getTopology().getAllEdgeIds())
    {
        const auto segments = curveSegmentManager.getSegmentsForEdge(edgeId);
        if (segments.empty())
            continue;

        std::vector<size_t> nodeIds;
        nodeIds.push_back(segments[0].nodeId1);
        for (const auto& segment : segments)
            nodeIds.push_back(segment.nodeId2);

        surfaceMesh.edgeNodeIds[edgeId] = std::move(nodeIds);
    }

    spdlog::debug("RCDTMesher::buildSurfaceMesh: {} nodes, {} triangles, {} faces, {} edges",
                  surfaceMesh.nodes.size(), surfaceMesh.triangles.size(),
                  surfaceMesh.faceTriangleIds.size(), surfaceMesh.edgeNodeIds.size());

    return surfaceMesh;
}

} // namespace Meshing
