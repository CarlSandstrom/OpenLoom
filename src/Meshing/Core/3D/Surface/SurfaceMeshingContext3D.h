#pragma once

#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include <memory>

namespace Geometry3D
{
class GeometryCollection3D;
} // namespace Geometry3D

namespace Topology3D
{
class Topology3D;
} // namespace Topology3D

class TwinManager;

namespace Meshing
{

class DiscretizationResult3D;
class FacetTriangulationManager;

/**
 * @brief Surface meshing context: owns all S1 infrastructure for the 3D surface mesher.
 *
 * Construction runs the full S1 initialization pipeline:
 *   1. TwinTableGenerator::generate(topology)         → EdgeTwinTable
 *   2. BoundaryDiscretizer3D::discretize()            → DiscretizationResult3D + TwinManager
 *   3. FacetTriangulationManager::createForSurfaceMesher() → per-face MeshData2D
 *
 * No tetrahedral mesh data is created or owned here.
 */
class SurfaceMeshingContext3D
{
public:
    /**
     * @brief Construct and fully initialize the surface meshing context.
     *
     * @param geometry  CAD geometry collection
     * @param topology  CAD topology
     * @param settings  Discretization settings (segments per edge, surface samples)
     */
    SurfaceMeshingContext3D(const Geometry3D::GeometryCollection3D& geometry,
                            const Topology3D::Topology3D& topology,
                            const Geometry3D::DiscretizationSettings3D& settings = {});

    ~SurfaceMeshingContext3D();

    // Prevent copying
    SurfaceMeshingContext3D(const SurfaceMeshingContext3D&) = delete;
    SurfaceMeshingContext3D& operator=(const SurfaceMeshingContext3D&) = delete;

    // Allow moving
    SurfaceMeshingContext3D(SurfaceMeshingContext3D&&) noexcept;
    SurfaceMeshingContext3D& operator=(SurfaceMeshingContext3D&&) noexcept;

    const Geometry3D::GeometryCollection3D& getGeometry() const { return *geometry_; }
    const Topology3D::Topology3D& getTopology() const { return *topology_; }

    const DiscretizationResult3D& getDiscretizationResult() const;
    const TwinManager& getTwinManager() const;
    FacetTriangulationManager& getFacetTriangulationManager();
    const FacetTriangulationManager& getFacetTriangulationManager() const;

private:
    const Geometry3D::GeometryCollection3D* geometry_;
    const Topology3D::Topology3D* topology_;

    std::unique_ptr<DiscretizationResult3D> discretizationResult_;
    std::unique_ptr<TwinManager> twinManager_;
    std::unique_ptr<FacetTriangulationManager> facetTriangulationManager_;
};

} // namespace Meshing
