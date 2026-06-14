#include "Meshing/Core/3D/RCDT/RCDTMesher.h"

#include "Common/Exceptions/GeometryException.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Meshing/Core/3D/General/BoundaryDiscretizer3D.h"
#include "Meshing/Core/3D/General/DiscretizationResult3D.h"
#include "Meshing/Core/3D/General/MeshDebugUtils3D.h"
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
    geometry_(&geometry),
    topology_(&topology),
    discretizationSettings_(discretizationSettings),
    qualitySettings_(qualitySettings),
    meshingContext_(std::make_unique<MeshingContext3D>(geometry, topology))
{
}

RCDTMesher::~RCDTMesher() = default;
RCDTMesher::RCDTMesher(RCDTMesher&&) noexcept = default;
RCDTMesher& RCDTMesher::operator=(RCDTMesher&&) noexcept = default;

SurfaceMesh3D RCDTMesher::mesh()
{
    size_t counter = 0;
    buildInitial();
    Meshing::exportMesh3D(meshingContext_->getMeshData(), "rcdt_initial", counter);

    refine();
    Meshing::exportMesh3D(meshingContext_->getMeshData(), "rcdt_refined", counter);

    return buildSurfaceMesh();
}

const MeshingContext3D& RCDTMesher::getMeshingContext() const
{
    OPENLOOM_REQUIRE_NOT_NULL(meshingContext_.get(), "meshingContext_");
    return *meshingContext_;
}

void RCDTMesher::buildInitial()
{
    spdlog::info("RCDTMesher::buildInitial: discretizing boundary (edges + corners only)");

    const Geometry3D::DiscretizationSettings3D edgeOnlySettings(
        discretizationSettings_.getNumSegmentsPerEdge(),
        discretizationSettings_.getMaxAngleBetweenSegments(),
        0);

    BoundaryDiscretizer3D discretizer(*geometry_, *topology_, edgeOnlySettings);
    discretizer.discretize();
    auto discretizationResult = discretizer.releaseDiscretizationResult();

    spdlog::info("RCDTMesher::buildInitial: {} points after discretization",
                 discretizationResult->points.size());

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

    auto& meshData = meshingContext_->getMeshData();

    Delaunay3D delaunay(discretizationResult->points, &meshData, enrichedGeometryIds);
    delaunay.triangulate();
    const auto pointIndexToNodeIdMap = delaunay.getPointIndexToNodeIdMap();

    spdlog::info("RCDTMesher::buildInitial: Delaunay3D produced {} nodes, {} elements",
                 meshData.getNodeCount(), meshData.getElementCount());

    meshingContext_->rebuildConnectivity();

    restrictedTriangulation_ = std::make_unique<RestrictedTriangulation>();
    const MeshConnectivity connectivity(meshData);
    restrictedTriangulation_->buildFrom(meshData, connectivity, *geometry_, *topology_);

    spdlog::info("RCDTMesher::buildInitial: {} restricted faces",
                 restrictedTriangulation_->getRestrictedFaces().size());

    CurveSegmentManager temporarySegmentManager;
    buildCurveSegments(temporarySegmentManager, *topology_, *geometry_,
                       discretizationResult->edgeIdToPointIndicesMap,
                       pointIndexToNodeIdMap,
                       discretizationResult->edgeParameters);

    auto& mutator = meshingContext_->getMutator();
    for (const auto& [segmentId, segment] : temporarySegmentManager.getAllSegments())
        mutator.addCurveSegment(segment);

    spdlog::info("RCDTMesher::buildInitial: {} curve segments added",
                 meshData.getCurveSegmentManager().size());
}

void RCDTMesher::refine()
{
    spdlog::info("RCDTMesher::refine: starting RCDT refinement");
    RCDTRefiner refiner(*meshingContext_, *restrictedTriangulation_, qualitySettings_);
    refiner.refine();
    spdlog::info("RCDTMesher::refine: done");
}

SurfaceMesh3D RCDTMesher::buildSurfaceMesh() const
{
    SurfaceMesh3D surfaceMesh;
    const auto& meshData = meshingContext_->getMeshData();

    if (meshData.getNodeCount() > 0)
    {
        size_t maxNodeId = 0;
        for (const auto& [nodeId, node] : meshData.getNodes())
            maxNodeId = std::max(maxNodeId, nodeId);

        surfaceMesh.nodes.resize(maxNodeId + 1);
        for (const auto& [nodeId, node] : meshData.getNodes())
            surfaceMesh.nodes[nodeId] = node->getCoordinates();
    }

    for (const auto& [faceKey, surfaceId] : restrictedTriangulation_->getRestrictedFaces())
    {
        const size_t triangleIndex = surfaceMesh.triangles.size();
        surfaceMesh.triangles.push_back({faceKey.nodeIds[0], faceKey.nodeIds[1], faceKey.nodeIds[2]});
        surfaceMesh.faceTriangleIds[surfaceId].push_back(triangleIndex);
    }

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

        surfaceMesh.edgeNodeIds[edgeId] = std::move(nodeIds);
    }

    spdlog::debug("RCDTMesher::buildSurfaceMesh: {} nodes, {} triangles, {} faces, {} edges",
                  surfaceMesh.nodes.size(), surfaceMesh.triangles.size(),
                  surfaceMesh.faceTriangleIds.size(), surfaceMesh.edgeNodeIds.size());

    return surfaceMesh;
}

} // namespace Meshing
