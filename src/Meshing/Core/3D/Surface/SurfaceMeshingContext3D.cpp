#include "Meshing/Core/3D/Surface/SurfaceMeshingContext3D.h"

#include "Common/TwinManager.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Core/2D/GeometryStructures2D.h"
#include "Meshing/Core/2D/MeshOperations2D.h"
#include "Meshing/Core/2D/MeshingContext2D.h"
#include "Meshing/Core/2D/Mesh2DQualitySettings.h"
#include "Meshing/Core/2D/ShewchukRefiner2D.h"
#include "Meshing/Core/3D/General/BoundaryDiscretizer3D.h"
#include "Meshing/Core/3D/General/DiscretizationResult3D.h"
#include "Meshing/Core/3D/General/FacetTriangulationManager.h"
#include "Meshing/Core/3D/General/MeshDebugUtils3D.h"
#include "Meshing/Core/3D/Surface/SurfaceMeshQuality.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include "Topology/Topology3D.h"
#include "spdlog/spdlog.h"

namespace Meshing
{

SurfaceMeshingContext3D::SurfaceMeshingContext3D(const Geometry3D::GeometryCollection3D& geometryCollection3D,
                                                 const Topology3D::Topology3D& topology3D,
                                                 const Geometry3D::DiscretizationSettings3D& discretizationSettings3D,
                                                 const SurfaceMesh3DQualitySettings& qualitySettings) :
    geometryCollection3D_(&geometryCollection3D),
    topology_(&topology3D),
    qualitySettings_(qualitySettings)
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

SurfaceMesh3D SurfaceMeshingContext3D::buildSurfaceMesh() const
{
    SurfaceMesh3D mesh;

    // Nodes: discretization points (IDs 0..N-1) followed by refinement nodes.
    mesh.nodes.insert(mesh.nodes.end(),
                      discretizationResult_->points.begin(),
                      discretizationResult_->points.end());
    mesh.nodes.insert(mesh.nodes.end(), refinementNodes_.begin(), refinementNodes_.end());

    // Triangles and per-face groups: iterate all surfaces in the facet manager
    // so we cover surfaces that have no interior sample points.
    for (const auto& surfaceId : facetTriangulationManager_->getSurfaceIds())
    {
        const size_t firstTriangleIndex = mesh.triangles.size();
        for (const auto& subfacet : facetTriangulationManager_->getSubfacetsForSurface(surfaceId))
        {
            mesh.triangles.push_back({subfacet.nodeId1, subfacet.nodeId2, subfacet.nodeId3});
        }
        const size_t lastTriangleIndex = mesh.triangles.size();

        if (lastTriangleIndex > firstTriangleIndex)
        {
            auto& group = mesh.faceTriangleIds[surfaceId];
            group.reserve(lastTriangleIndex - firstTriangleIndex);
            for (size_t i = firstTriangleIndex; i < lastTriangleIndex; ++i)
                group.push_back(i);
        }
    }

    // Edge node sequences (post-refinement, includes split nodes).
    mesh.edgeNodeIds = facetTriangulationManager_->buildEdgeNodeIds();

    spdlog::debug("SurfaceMeshingContext3D::buildSurfaceMesh: {} nodes, {} triangles, {} faces, {} edges",
                  mesh.nodes.size(), mesh.triangles.size(),
                  mesh.faceTriangleIds.size(), mesh.edgeNodeIds.size());

    return mesh;
}

void SurfaceMeshingContext3D::refineSurfaces()
{
    const double circumradiusToEdgeRatio = qualitySettings_.circumradiusToEdgeRatio;
    const double minAngleDegrees = qualitySettings_.minAngleDegrees;
    const size_t elementLimit = qualitySettings_.elementLimit;
    const double chordDeviationTolerance = qualitySettings_.chordDeviationTolerance;
    auto& manager = *facetTriangulationManager_;

    refinementNodes_.clear();
    size_t nextNode3DId = discretizationResult_->points.size();

    // Apply a cross-face boundary split: split segment (m1,m2) on the twin face
    // and record the split in the TwinManager so sub-splits find their twins.
    auto applyCrossFaceSplit = [&](FacetTriangulation& twinFacet,
                                   const std::string& twinSurfaceId,
                                   const std::string& sourceSurfaceId,
                                   size_t n1, size_t n2, size_t mid,
                                   size_t node3DId,
                                   size_t m1, size_t m2)
    {
        MeshingContext2D& twinContext = twinFacet.getContext();
        auto twinEdgeId = twinContext.getOperations().getQueries().findCommonGeometryId(m1, m2);
        if (!twinEdgeId) return;
        const auto* twinEdge = twinContext.getGeometry().getEdge(*twinEdgeId);
        if (!twinEdge) return;
        ConstrainedSegment2D twinSeg{m1, m2};
        auto twinMidOpt = twinContext.getOperations().splitConstrainedSegment(twinSeg, *twinEdge);
        if (!twinMidOpt) return;
        size_t twinMid = *twinMidOpt;
        twinFacet.registerNode(twinMid, node3DId);
        twinFacet.updateEdgeNodeAfterSplit(m1, m2, twinMid);
        twinManager_->recordSplit(sourceSurfaceId, n1, n2, mid, twinSurfaceId, m1, m2, twinMid);
    };

    // Refine one face: set up the boundary-split callback, run Shewchuk, resolve nodes.
    // Cross-face splits are always applied immediately to the twin face so the
    // TwinManager stays current for sub-splits. Returns true if any cross-face
    // split occurred (so the outer loop knows whether to run another pass).
    //
    // deviationPassOnly = false  → angle-quality pass (Shewchuk2DQualityController, UV space)
    // deviationPassOnly = true   → chord-deviation pass (SurfaceMeshQualityController,
    //                              angle bounds disabled, only chord height checked)
    auto refineOneFace = [&](const std::string& surfaceId, FacetTriangulation* facetTriang,
                             bool deviationPassOnly) -> bool
    {
        spdlog::info("SurfaceMeshingContext3D::refineSurfaces: refining surface '{}' ({})",
                     surfaceId, deviationPassOnly ? "deviation" : "angle");

        MeshingContext2D& faceContext = facetTriang->getContext();

        std::unique_ptr<ShewchukRefiner2D> refiner;
        if (!deviationPassOnly)
        {
            Mesh2DQualitySettings settings;
            settings.circumradiusToEdgeRatio = circumradiusToEdgeRatio;
            settings.minAngleDegrees         = minAngleDegrees;
            settings.elementLimit            = elementLimit;
            refiner = std::make_unique<ShewchukRefiner2D>(faceContext, settings, surfaceId);
        }
        else
        {
            const auto* surface = geometryCollection3D_->getSurface(surfaceId);
            if (!surface)
                return false;
            // Use very loose angle bounds so only chord deviation triggers refinement.
            auto controller = std::make_unique<SurfaceMeshQualityController>(
                faceContext.getMeshData(), *surface,
                1e9, // circumradius bound: effectively disabled
                0.0, // min angle: effectively disabled
                elementLimit,
                chordDeviationTolerance);
            refiner = std::make_unique<ShewchukRefiner2D>(
                faceContext, std::move(controller), surfaceId);
        }

        bool hadCrossFaceSplit = false;

        refiner->setOnBoundarySplit([&, surfaceId, facetTriang](size_t n1, size_t n2, size_t mid)
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
                // Seam twin (same surface): split immediately.
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
            else
            {
                // Cross-surface twin: apply immediately and flag the outer loop
                // to run another pass (the twin may need re-refinement).
                auto* twinFacet = manager.getFacetTriangulation(twinSurfaceId);
                if (twinFacet)
                {
                    applyCrossFaceSplit(*twinFacet, twinSurfaceId, surfaceId,
                                       n1, n2, mid, node3DId, m1, m2);
                    hadCrossFaceSplit = true;
                }
            } });

        refiner->refine();

        for (const auto& pt : facetTriang->resolveRefinementNodes(nextNode3DId))
            refinementNodes_.push_back(pt);

        return hadCrossFaceSplit;
    };

    // Phase 1: Angle-quality passes (UV space). Repeat until a full pass produces
    // no cross-face boundary splits.
    bool anyNewSplits = true;
    while (anyNewSplits)
    {
        anyNewSplits = false;
        for (const auto& surfaceId : topology_->getAllSurfaceIds())
        {
            auto* facetTriang = manager.getFacetTriangulation(surfaceId);
            if (!facetTriang)
                continue;
            if (refineOneFace(surfaceId, facetTriang, false))
                anyNewSplits = true;
        }
    }

    // Phase 2: Chord-deviation passes (optional). Refine any triangle whose chord
    // height exceeds the tolerance, using the same pass-until-stable strategy.
    if (chordDeviationTolerance > 0.0)
    {
        spdlog::info("SurfaceMeshingContext3D::refineSurfaces: starting chord-deviation pass "
                     "(tolerance {})",
                     chordDeviationTolerance);

        bool anyDeviationSplits = true;
        while (anyDeviationSplits)
        {
            anyDeviationSplits = false;
            for (const auto& surfaceId : topology_->getAllSurfaceIds())
            {
                auto* facetTriang = manager.getFacetTriangulation(surfaceId);
                if (!facetTriang)
                    continue;
                if (refineOneFace(surfaceId, facetTriang, true))
                    anyDeviationSplits = true;
            }
        }
    }

    spdlog::info("SurfaceMeshingContext3D::refineSurfaces: complete ({} refinement nodes)",
                 refinementNodes_.size());
}

} // namespace Meshing
