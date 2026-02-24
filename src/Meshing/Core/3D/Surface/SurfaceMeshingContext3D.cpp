#include "Meshing/Core/3D/Surface/SurfaceMeshingContext3D.h"

#include "Common/TwinManager.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Meshing/Core/3D/General/BoundaryDiscretizer3D.h"
#include "Meshing/Core/3D/General/DiscretizationResult3D.h"
#include "Meshing/Core/3D/Surface/FacetTriangulationManager.h"
#include "Meshing/Core/3D/Surface/TwinTableGenerator.h"
#include "Topology/Topology3D.h"
#include "spdlog/spdlog.h"

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

} // namespace Meshing
