#include "Meshing/Core/3D/Surface/SurfaceMeshingContext3D.h"

#include "Common/TwinManager.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Core/2D/GeometryStructures2D.h"
#include "Meshing/Core/2D/MeshOperations2D.h"
#include "Meshing/Core/2D/MeshingContext2D.h"
#include "Meshing/Core/2D/Shewchuk2DQualityController.h"
#include "Meshing/Core/2D/ShewchukRefiner2D.h"
#include "Meshing/Core/3D/General/BoundaryDiscretizer3D.h"
#include "Meshing/Core/3D/General/DiscretizationResult3D.h"
#include "Meshing/Core/3D/General/FacetTriangulationManager.h"
#include "Meshing/Core/3D/General/MeshDebugUtils3D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Topology/Topology3D.h"
#include "spdlog/spdlog.h"
#include <map>
#include <set>
#include <numbers>

namespace Meshing
{

SurfaceMeshingContext3D::SurfaceMeshingContext3D(const Geometry3D::GeometryCollection3D& geometryCollection3D,
                                                 const Topology3D::Topology3D& topology3D,
                                                 const Geometry3D::DiscretizationSettings3D& discretizationSettings3D) :
    geometryCollection3D_(&geometryCollection3D),
    topology_(&topology3D)
{
    spdlog::info("SurfaceMeshingContext3D: initializing S1 pipeline");

    // S1.2: Discretize boundaries
    BoundaryDiscretizer3D boundaryDiscretizer3D(geometryCollection3D, topology3D, discretizationSettings3D);
    boundaryDiscretizer3D.discretize();
    discretizationResult_ = boundaryDiscretizer3D.releaseDiscretizationResult();

    exportEdgeMesh3D(*discretizationResult_, "SurfaceMeshingContext3D_edges.vtu");

    spdlog::info("SurfaceMeshingContext3D: discretized {} points", discretizationResult_->points.size());

    // S1.4: Initialize per-face triangulations and build surface-aware TwinManager
    auto manager = FacetTriangulationManager::createForSurfaceMesher(geometryCollection3D, topology3D, *discretizationResult_);
    facetTriangulationManager_ = std::make_unique<FacetTriangulationManager>(std::move(manager));
    twinManager_ = facetTriangulationManager_->releaseTwinManager();

    spdlog::info("SurfaceMeshingContext3D: initialized {} facet triangulations", facetTriangulationManager_->size());
}

SurfaceMeshingContext3D::~SurfaceMeshingContext3D() = default;

SurfaceMeshingContext3D::SurfaceMeshingContext3D(SurfaceMeshingContext3D&&) noexcept = default;
SurfaceMeshingContext3D& SurfaceMeshingContext3D::operator=(SurfaceMeshingContext3D&&) noexcept = default;

const DiscretizationResult3D& SurfaceMeshingContext3D::getDiscretizationResult() const
{
    return *discretizationResult_;
}

const TwinManager& SurfaceMeshingContext3D::getTwinManager() const
{
    return *twinManager_;
}

FacetTriangulationManager& SurfaceMeshingContext3D::getFacetTriangulationManager()
{
    return *facetTriangulationManager_;
}

const FacetTriangulationManager& SurfaceMeshingContext3D::getFacetTriangulationManager() const
{
    return *facetTriangulationManager_;
}

MeshData3D SurfaceMeshingContext3D::getSurfaceMesh3D() const
{
    MeshData3D mesh;
    MeshMutator3D mutator(mesh);

    // Add one node per discretization point (IDs 0..N-1).
    for (const auto& pt : discretizationResult_->points)
        mutator.addNode(pt);

    // Add refinement nodes resolved during refineSurfaces() (IDs N..M-1).
    for (const auto& pt : refinementNodes_)
        mutator.addNode(pt);

    // Add one triangle element per subfacet from all facet triangulations.
    for (const auto& subfacet : facetTriangulationManager_->getAllSubfacets())
    {
        mutator.addElement(std::make_unique<TriangleElement>(
            std::array<size_t, 3>{subfacet.nodeId1, subfacet.nodeId2, subfacet.nodeId3}));
    }

    spdlog::debug("SurfaceMeshingContext3D::buildSurfaceMesh: {} nodes, {} triangles",
                  mesh.getNodeCount(), mesh.getElementCount());

    return mesh;
}

void SurfaceMeshingContext3D::refineSurfaces(double circumradiusToEdgeRatio,
                                             double minAngleDegrees,
                                             size_t elementLimit)
{
    const double minAngleRadians = minAngleDegrees * (std::numbers::pi / 180.0);
    auto& manager = *facetTriangulationManager_;

    refinementNodes_.clear();
    size_t nextNode3DId = discretizationResult_->points.size();

    // Cross-face splits targeting an already-refined face cannot be applied
    // immediately: the refiner for that face has finished, and re-running it is
    // out of scope. Collect them here and apply them after all faces are refined
    // (geometry stays consistent; quality of the late triangles is not re-checked).
    //
    // Splits targeting a face that has NOT yet been refined are applied immediately
    // inside the callback — this is safe because that face's refiner hasn't started
    // yet, and immediate application keeps the TwinManager current so that further
    // sub-splits of the same segment also find their twins.
    struct PendingCrossFaceSplit
    {
        std::string sourceSurfaceId;
        size_t n1, n2, mid;
        size_t node3DId;
        size_t m1, m2;
    };
    std::set<std::string> refinedSurfaces;
    std::map<std::string, std::vector<PendingCrossFaceSplit>> pendingCrossFaceSplits;

    auto applyPendingSplit = [&](FacetTriangulation& targetFacet,
                                 const std::string& twinSurfaceId,
                                 const PendingCrossFaceSplit& pending)
    {
        MeshingContext2D& twinContext = targetFacet.getContext();
        auto twinEdgeId = twinContext.getOperations().getQueries().findCommonGeometryId(pending.m1, pending.m2);
        if (!twinEdgeId)
            return;
        const auto* twinEdge = twinContext.getGeometry().getEdge(*twinEdgeId);
        if (!twinEdge)
            return;
        ConstrainedSegment2D twinSeg{pending.m1, pending.m2};
        auto twinMidOpt = twinContext.getOperations().splitConstrainedSegment(twinSeg, *twinEdge);
        if (!twinMidOpt)
            return;
        size_t twinMid = *twinMidOpt;
        targetFacet.registerNode(twinMid, pending.node3DId);
        targetFacet.updateEdgeNodeAfterSplit(pending.m1, pending.m2, twinMid);
        twinManager_->recordSplit(pending.sourceSurfaceId, pending.n1, pending.n2, pending.mid,
                                  twinSurfaceId, pending.m1, pending.m2, twinMid);
    };

    // Refine one face: set up the boundary-split callback, run Shewchuk, resolve nodes.
    // Cross-face splits targeting not-yet-refined faces are applied immediately so the
    // TwinManager stays current for sub-splits. Splits targeting already-refined faces
    // are deferred into pendingCrossFaceSplits for the iterative re-refinement below.
    auto refineOneFace = [&](const std::string& surfaceId, FacetTriangulation* facetTriang)
    {
        spdlog::info("SurfaceMeshingContext3D::refineSurfaces: refining surface '{}'", surfaceId);

        MeshingContext2D& faceContext = facetTriang->getContext();

        Shewchuk2DQualityController qualityController(faceContext.getMeshData(),
                                                      circumradiusToEdgeRatio,
                                                      minAngleRadians,
                                                      elementLimit);

        ShewchukRefiner2D refiner(faceContext, qualityController, surfaceId);

        refiner.setOnBoundarySplit([&, surfaceId, facetTriang](size_t n1, size_t n2, size_t mid)
                                   {
            auto twinOpt = twinManager_->getTwin(surfaceId, n1, n2);
            if (!twinOpt)
                return;

            auto [twinSurfaceId, m1, m2] = *twinOpt;

            // Compute 3D position from source node's UV. The split point lies on
            // a shared 3D edge so both surfaces evaluate to the same 3D location.
            const auto* sourceNode = faceContext.getMeshData().getNode(mid);
            if (!sourceNode)
                return;

            const auto& uv = sourceNode->getCoordinates();
            const auto* sourceSurface = geometryCollection3D_->getSurface(surfaceId);
            if (!sourceSurface)
                return;

            size_t node3DId = nextNode3DId++;
            refinementNodes_.push_back(sourceSurface->getPoint(uv.x(), uv.y()));
            facetTriang->registerNode(mid, node3DId);
            facetTriang->updateEdgeNodeAfterSplit(n1, n2, mid);

            if (twinSurfaceId == surfaceId)
            {
                // Seam twin (same surface): split immediately — safe to call into
                // the same triangulation context while the refiner is running.
                auto twinEdgeId = faceContext.getOperations().getQueries().findCommonGeometryId(m1, m2);
                if (!twinEdgeId)
                    return;
                const auto* twinEdge = faceContext.getGeometry().getEdge(*twinEdgeId);
                if (!twinEdge)
                    return;
                ConstrainedSegment2D twinSeg{m1, m2};
                auto twinMidOpt = faceContext.getOperations().splitConstrainedSegment(twinSeg, *twinEdge);
                if (!twinMidOpt)
                    return;
                size_t twinMid = *twinMidOpt;
                facetTriang->registerNode(twinMid, node3DId);
                facetTriang->updateEdgeNodeAfterSplit(m1, m2, twinMid);
                twinManager_->recordSplit(surfaceId, n1, n2, mid, twinSurfaceId, m1, m2, twinMid);
            }
            else if (refinedSurfaces.count(twinSurfaceId) == 0)
            {
                // Twin face not yet refined: apply immediately so the TwinManager
                // stays current for any further sub-splits of this segment.
                auto* twinFacet = manager.getFacetTriangulation(twinSurfaceId);
                if (twinFacet)
                    applyPendingSplit(*twinFacet, twinSurfaceId,
                                     {surfaceId, n1, n2, mid, node3DId, m1, m2});
            }
            else
            {
                // Twin face already refined: defer so it can be re-refined in the
                // next iteration of the late-split loop below.
                pendingCrossFaceSplits[twinSurfaceId].push_back(
                    {surfaceId, n1, n2, mid, node3DId, m1, m2});
            } });

        refiner.refine();

        for (const auto& pt : facetTriang->resolveRefinementNodes(nextNode3DId))
            refinementNodes_.push_back(pt);
    };

    // Initial pass: refine every face in topology order.
    for (const auto& surfaceId : topology_->getAllSurfaceIds())
    {
        auto* facetTriang = manager.getFacetTriangulation(surfaceId);
        if (!facetTriang)
            continue;

        // Apply any cross-face splits that arrived for this face before it runs.
        auto pendingIt = pendingCrossFaceSplits.find(surfaceId);
        if (pendingIt != pendingCrossFaceSplits.end())
        {
            for (const auto& pending : pendingIt->second)
                applyPendingSplit(*facetTriang, surfaceId, pending);
            pendingCrossFaceSplits.erase(pendingIt);
        }

        refineOneFace(surfaceId, facetTriang);
        refinedSurfaces.insert(surfaceId);
    }

    // Iterative late-split resolution: any face that received a deferred split
    // gets it applied and is then re-refined so the new boundary nodes don't leave
    // large unsplit triangles. Re-refinement may generate new deferred splits for
    // other already-refined faces, so we loop until the queue empties.
    while (!pendingCrossFaceSplits.empty())
    {
        auto batch = std::move(pendingCrossFaceSplits);
        pendingCrossFaceSplits.clear();

        for (auto& [twinSurfaceId, splits] : batch)
        {
            auto* twinFacet = manager.getFacetTriangulation(twinSurfaceId);
            if (!twinFacet)
                continue;

            for (const auto& pending : splits)
                applyPendingSplit(*twinFacet, twinSurfaceId, pending);

            refineOneFace(twinSurfaceId, twinFacet);
        }
    }

    spdlog::info("SurfaceMeshingContext3D::refineSurfaces: complete ({} refinement nodes)",
                 refinementNodes_.size());
}

} // namespace Meshing
