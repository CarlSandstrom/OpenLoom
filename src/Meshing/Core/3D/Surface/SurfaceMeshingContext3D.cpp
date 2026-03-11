#include "Meshing/Core/3D/Surface/SurfaceMeshingContext3D.h"

#include "Common/TwinManager.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Meshing/Core/2D/BoundarySplitSynchronizer.h"
#include "Meshing/Core/2D/Shewchuk2DQualityController.h"
#include "Meshing/Core/2D/ShewchukRefiner2D.h"
#include "Meshing/Core/3D/General/BoundaryDiscretizer3D.h"
#include "Meshing/Core/3D/General/DiscretizationResult3D.h"
#include "Meshing/Core/3D/General/MeshDebugUtils3D.h"
#include "Meshing/Core/3D/Surface/FacetTriangulationManager.h"
#include "Meshing/Core/3D/Surface/TwinTableGenerator.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Topology/Topology3D.h"
#include "spdlog/spdlog.h"
#include <numbers>

namespace Meshing
{

SurfaceMeshingContext3D::SurfaceMeshingContext3D(const Geometry3D::GeometryCollection3D& geometry,
                                                 const Topology3D::Topology3D& topology,
                                                 const Geometry3D::DiscretizationSettings3D& settings) :
    geometry_(&geometry),
    topology_(&topology)
{
    spdlog::info("SurfaceMeshingContext3D: initializing S1 pipeline");

    // S1.2 → S1.3: Generate twin table, then discretize boundaries and populate TwinManager
    EdgeTwinTable twinTable = TwinTableGenerator::generate(topology);

    BoundaryDiscretizer3D discretizer(geometry, topology, settings, twinTable);
    discretizer.discretize();

    discretizationResult_ = discretizer.releaseDiscretizationResult();
    twinManager_ = discretizer.releaseTwinManager();

    exportEdgeMesh3D(*discretizationResult_, "SurfaceMeshingContext3D_edges.vtu");

    spdlog::info("SurfaceMeshingContext3D: discretized {} points, {} twin segments",
                 discretizationResult_->points.size(),
                 twinTable.size());

    // S1.4: Initialize per-face triangulations (surface-mesher path, no MeshData3D)
    auto manager = FacetTriangulationManager::createForSurfaceMesher(geometry, topology, *discretizationResult_);
    facetTriangulationManager_ = std::make_unique<FacetTriangulationManager>(std::move(manager));

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

    for (const auto& surfaceId : topology_->getAllSurfaceIds())
    {
        auto* facetTriang = manager.getFacetTriangulation(surfaceId);
        if (!facetTriang)
            continue;

        spdlog::info("SurfaceMeshingContext3D::refineSurfaces: refining surface '{}'", surfaceId);

        MeshingContext2D& faceContext = facetTriang->getContext();

        Shewchuk2DQualityController qualityController(
            faceContext.getMeshData(),
            circumradiusToEdgeRatio,
            minAngleRadians,
            elementLimit);

        ShewchukRefiner2D refiner(faceContext, qualityController);
        BoundarySplitSynchronizer sync(faceContext, *twinManager_);
        refiner.setOnBoundarySplit(sync);
        refiner.refine();
    }

    // Assign 3D node IDs to all 2D nodes inserted during refinement (they have no 3D peer
    // yet). This must happen before any getAllSubfacets() call so that getSubfacets() can
    // map every 2D node to a 3D ID without warnings.
    refinementNodes_.clear();
    size_t nextNode3DId = discretizationResult_->points.size();
    for (const auto& surfaceId : topology_->getAllSurfaceIds())
    {
        auto* facetTriang = manager.getFacetTriangulation(surfaceId);
        if (!facetTriang)
            continue;
        for (const auto& pt : facetTriang->resolveRefinementNodes(nextNode3DId))
            refinementNodes_.push_back(pt);
    }

    spdlog::info("SurfaceMeshingContext3D::refineSurfaces: complete ({} refinement nodes)",
                 refinementNodes_.size());
}

} // namespace Meshing
