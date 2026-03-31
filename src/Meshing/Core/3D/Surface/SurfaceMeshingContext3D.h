#pragma once

#include "Common/Types.h"
#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Data/3D/SurfaceMesh3D.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"
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
                            const Geometry3D::DiscretizationSettings3D& discretizationSettings = {},
                            const SurfaceMesh3DQualitySettings& qualitySettings = {});

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
     * @brief Run ShewchukRefiner2D on every face in UV space.
     *
     * Must be called after construction and before getSurfaceMesh3D().
     * Quality settings are taken from the SurfaceMesh3DQualitySettings passed
     * at construction time.
     *
     * Refinement runs in two phases:
     *
     * Phase 1 — Angle quality (UV space): passes over all faces using
     * Shewchuk2DQualityController until a full pass produces no cross-face
     * boundary splits. Within each pass, every boundary split fires a callback
     * that immediately applies the matching split to the twin face (seam twins on
     * the same surface, or cross-surface twins on an adjacent face).
     *
     * Phase 2 — Chord deviation (optional): if qualitySettings.chordDeviationTolerance > 0,
     * a second round of passes refines any triangle whose chord height exceeds the
     * tolerance. This ensures the triangulation geometrically approximates the curved
     * surface within the given tolerance regardless of triangle size.
     *
     * See doc/Flowcharts/3D surface meshing algorithm.md for full details.
     */
    void refineSurfaces();

    /**
     * @brief Assemble a SurfaceMesh3D from the current triangulation state.
     *
     * Collects all nodes (discretization points + refinement nodes), triangles,
     * per-face triangle groups, and per-edge ordered node sequences.
     *
     * Must be called after refineSurfaces().
     */
    SurfaceMesh3D buildSurfaceMesh() const;

private:
    const Geometry3D::GeometryCollection3D* geometryCollection3D_;
    const Topology3D::Topology3D* topology_;
    SurfaceMesh3DQualitySettings qualitySettings_;

    std::unique_ptr<DiscretizationResult3D> discretizationResult_;
    std::unique_ptr<TwinManager> twinManager_;
    std::unique_ptr<FacetTriangulationManager> facetTriangulationManager_;

    // 3D positions of nodes inserted during Shewchuk refinement (populated by refineSurfaces).
    // Their 3D node IDs follow immediately after discretizationResult_->points.
    std::vector<Point3D> refinementNodes_;
};

} // namespace Meshing
