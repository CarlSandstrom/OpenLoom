#pragma once

#include "Common/Types.h"
#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include <memory>
#include <vector>

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
 *   1. BoundaryDiscretizer3D::discretize()                 → DiscretizationResult3D
 *   2. FacetTriangulationManager::createForSurfaceMesher() → per-face MeshData2D + TwinManager
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

    const Geometry3D::GeometryCollection3D& getGeometry() const { return *geometryCollection3D_; }
    const Topology3D::Topology3D& getTopology() const { return *topology_; }

    const DiscretizationResult3D& getDiscretizationResult() const;
    const TwinManager& getTwinManager() const;
    FacetTriangulationManager& getFacetTriangulationManager();
    const FacetTriangulationManager& getFacetTriangulationManager() const;

    /**
     * @brief Run ShewchukRefiner2D on every face in UV space (Step S2.1).
     *
     * Must be called after construction and before buildSurfaceMesh().
     * Within-face seam-edge twins (e.g. cylindrical faces) are kept
     * synchronised via BoundarySplitSynchronizer. Cross-face twin
     * propagation is deferred to S3.2.
     *
     * @param circumradiusToEdgeRatio  Max circumradius/shortest-edge bound (default 2.0)
     * @param minAngleDegrees          Minimum interior angle in degrees (default 30.0)
     * @param elementLimit             Safety cap on elements per face (default 50000)
     */
    void refineSurfaces(double circumradiusToEdgeRatio = 2.0,
                        double minAngleDegrees = 30.0,
                        size_t elementLimit = 50000);

    /**
     * @brief Build a MeshData3D containing the surface triangulation.
     *
     * Adds one Node3D per discretization point (node ID == point index) plus any
     * nodes inserted during refineSurfaces(), then one TriangleElement per subfacet.
     */
    MeshData3D getSurfaceMesh3D() const;

private:
    const Geometry3D::GeometryCollection3D* geometryCollection3D_;
    const Topology3D::Topology3D* topology_;

    std::unique_ptr<DiscretizationResult3D> discretizationResult_;
    std::unique_ptr<TwinManager> twinManager_;
    std::unique_ptr<FacetTriangulationManager> facetTriangulationManager_;

    // 3D positions of nodes inserted during Shewchuk refinement (populated by refineSurfaces).
    // Their 3D node IDs follow immediately after discretizationResult_->points.
    std::vector<Point3D> refinementNodes_;
};

} // namespace Meshing
