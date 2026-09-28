#pragma once

#include "Common/TwinManager.h"
#include "Common/Types.h"
#include "Meshing/Core/3D/General/DiscretizationResult3D.h"
#include "Meshing/Core/3D/General/FacetTriangulation.h"
#include "Meshing/Core/3D/General/GeometryStructures3D.h"
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace Geometry3D
{
class GeometryCollection3D;
} // namespace Geometry3D

namespace Topology3D
{
class Topology3D;
} // namespace Topology3D

namespace Meshing
{

/**
 * @brief Manages all facet triangulations for 3D meshing
 *
 * Creates and maintains independent 2D triangulations for each surface in
 * the 3D domain, for the legacy PerFaceUV surface mesher.
 *
 * Also builds a surface-aware TwinManager after all facets are initialized.
 * The TwinManager maps segment pairs by their 2D node IDs so that boundary
 * splits during Shewchuk refinement can be propagated correctly across shared
 * or seam edges.
 *
 * Construction is via named factory methods — the object is always fully
 * initialised on creation and cannot be used in an empty state.
 */
class FacetTriangulationManager
{
public:
    ~FacetTriangulationManager();

    // Prevent copying
    FacetTriangulationManager(const FacetTriangulationManager&) = delete;
    FacetTriangulationManager& operator=(const FacetTriangulationManager&) = delete;

    // Allow moving
    FacetTriangulationManager(FacetTriangulationManager&&) noexcept;
    FacetTriangulationManager& operator=(FacetTriangulationManager&&) noexcept;

    // -----------------------------------------------------------------------
    // Factory methods
    // -----------------------------------------------------------------------

    /**
     * @brief Create a manager for the surface mesher.
     *
     * For each surface:
     * 1. Collects all points (corners + edge points + interior points)
     * 2. Projects them to (u,v) parametric space via ISurface3D::projectPoint()
     * 3. Creates a 2D Delaunay triangulation
     *
     * After all facets are initialized, builds a surface-aware TwinManager from
     * the edge-to-node sequences stored in each FacetTriangulation.  The manager
     * can then be released via releaseTwinManager().
     *
     * Point indices in @p discretization are used directly as "3D node IDs".
     */
    static FacetTriangulationManager createForSurfaceMesher(const Geometry3D::GeometryCollection3D& geometry,
                                                            const Topology3D::Topology3D& topology,
                                                            const DiscretizationResult3D& discretization);

    // -----------------------------------------------------------------------
    // Twin manager
    // -----------------------------------------------------------------------

    /**
     * @brief Transfer ownership of the TwinManager to the caller.
     *
     * The caller (SurfaceMeshingContext3D) must hold the returned unique_ptr
     * for the full meshing lifetime.
     *
     * Returns nullptr if called twice.
     */
    std::unique_ptr<TwinManager> releaseTwinManager();

    // -----------------------------------------------------------------------
    // Queries
    // -----------------------------------------------------------------------

    /**
     * @brief Get all subfacets from all facet triangulations
     * @return Vector of all ConstrainedSubfacet3D
     */
    std::vector<ConstrainedSubfacet3D> getAllSubfacets() const;

    /**
     * @brief Get subfacets for a specific surface
     * @param surfaceId The surface ID
     * @return Vector of ConstrainedSubfacet3D for that surface
     */
    std::vector<ConstrainedSubfacet3D> getSubfacetsForSurface(const std::string& surfaceId) const;

    /**
     * @brief Get the facet triangulation for a surface
     * @param surfaceId The surface ID
     * @return Pointer to the FacetTriangulation, or nullptr if not found
     */
    FacetTriangulation* getFacetTriangulation(const std::string& surfaceId);
    const FacetTriangulation* getFacetTriangulation(const std::string& surfaceId) const;

    /**
     * @brief Build the mapping from CAD edge ID to ordered 3D node IDs along that edge.
     *
     * Iterates all facet triangulations and collects each edge's 3D node sequence
     * (post-refinement, so split nodes are included).  When an edge is shared by
     * two faces both sequences agree after twin synchronisation; the first face
     * encountered for each edge ID is used.
     *
     * Intended to populate SurfaceMesh3D::edgeNodeIds after refinement is complete.
     */
    std::map<std::string, std::vector<size_t>> buildEdgeNodeIds() const;

    /**
     * @brief Get all surface IDs managed by this instance.
     */
    std::vector<std::string> getSurfaceIds() const;

    /**
     * @brief Get number of facet triangulations
     */
    size_t size() const { return facetTriangulations_.size(); }

private:
    FacetTriangulationManager(const Geometry3D::GeometryCollection3D& geometry,
                              const Topology3D::Topology3D& topology);

    void initializeForSurfaceMesher(const DiscretizationResult3D& discretization);

    /**
     * @brief Build a surface-aware TwinManager from the initialized facet triangulations.
     *
     * Registers two kinds of twin pairs using local 2D node IDs:
     *   1. Cross-surface twins: edges shared between two different surfaces
     *      (from the EdgeTwinTable generated by TwinTableGenerator).
     *   2. Seam twins: the two sides of a periodic seam on the same surface
     *      (from SeamCollection), where the twin sequence runs in reverse.
     *
     * Must be called after all facets have been initialized so that every
     * FacetTriangulation has its edge-to-node-sequence map populated.
     */
    void buildTwinManager();

    /**
     * @brief Collect boundary-only point indices for a surface (corners + edge points).
     *
     * Excludes interior surface points so the surface mesher starts with only
     * boundary vertices and lets refinement add interior points.
     */
    std::vector<size_t> collectBoundaryPointIndices(const std::string& surfaceId,
                                                    const DiscretizationResult3D& discretization) const;

    const Geometry3D::GeometryCollection3D* geometry_;
    const Topology3D::Topology3D* topology_;

    // Map from surface ID to its facet triangulation
    std::unordered_map<std::string, std::unique_ptr<FacetTriangulation>> facetTriangulations_;

    // Built by initializeForSurfaceMesher(); released via releaseTwinManager().
    std::unique_ptr<TwinManager> twinManager_;
};

} // namespace Meshing
