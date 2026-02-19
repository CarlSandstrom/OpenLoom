#pragma once

#include "Common/Types.h"
#include "Meshing/Core/3D/General/DiscretizationResult3D.h"
#include "Meshing/Core/3D/Surface/FacetTriangulation.h"
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

class MeshData3D;

/**
 * @brief Manages all facet triangulations for 3D constrained Delaunay meshing
 *
 * Creates and maintains independent 2D triangulations for each surface in
 * the 3D domain. These triangulations define the initial subfacet constraints
 * and are updated when vertices are inserted during refinement.
 *
 * Per Shewchuk's algorithm, facet triangulations are maintained separately
 * from the 3D tetrahedralization and define what subfacets *should* exist.
 */
class FacetTriangulationManager
{
public:
    /**
     * @brief Construct manager with geometry and topology
     */
    FacetTriangulationManager(const Geometry3D::GeometryCollection3D& geometry,
                               const Topology3D::Topology3D& topology);

    ~FacetTriangulationManager();

    // Prevent copying
    FacetTriangulationManager(const FacetTriangulationManager&) = delete;
    FacetTriangulationManager& operator=(const FacetTriangulationManager&) = delete;

    // Allow moving
    FacetTriangulationManager(FacetTriangulationManager&&) noexcept;
    FacetTriangulationManager& operator=(FacetTriangulationManager&&) noexcept;

    /**
     * @brief Initialize all facet triangulations from discretization result
     *
     * For each surface:
     * 1. Collects all points (corners + edge points + interior points)
     * 2. Projects them to (u,v) parametric space using ISurface3D::projectPoint()
     * 3. Creates 2D Delaunay triangulation
     * 4. Stores triangles as ConstrainedSubfacet3D
     *
     * @param discretization The boundary discretization result
     * @param pointIndexToNodeIdMap Maps discretization point indices to 3D node IDs
     * @param meshData The 3D mesh data for node coordinate lookups
     */
    void initializeFromDiscretization(
        const DiscretizationResult3D& discretization,
        const std::map<size_t, size_t>& pointIndexToNodeIdMap,
        const MeshData3D& meshData);

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
     * @brief Insert a vertex into the appropriate facet triangulation
     *
     * Determines which surface the vertex belongs to and updates that
     * facet's triangulation. Used during refinement when splitting subfacets.
     *
     * @param node3DId The 3D mesh node ID
     * @param point The 3D point coordinates
     * @param surfaceId The surface this vertex lies on
     * @return true if insertion succeeded
     */
    bool insertVertexOnSurface(size_t node3DId, const Point3D& point,
                               const std::string& surfaceId);

    /**
     * @brief Get number of facet triangulations
     */
    size_t size() const { return facetTriangulations_.size(); }

private:
    const Geometry3D::GeometryCollection3D* geometry_;
    const Topology3D::Topology3D* topology_;

    // Map from surface ID to its facet triangulation
    std::unordered_map<std::string, std::unique_ptr<FacetTriangulation>> facetTriangulations_;

    /**
     * @brief Collect all point indices belonging to a surface
     *
     * Includes corner points, edge points (from boundary edges), and
     * interior surface points.
     */
    std::vector<size_t> collectSurfacePointIndices(
        const std::string& surfaceId,
        const DiscretizationResult3D& discretization) const;
};

} // namespace Meshing
