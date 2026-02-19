#pragma once

#include "Common/Types.h"
#include "Meshing/Core/3D/General/GeometryStructures3D.h"
#include "Meshing/Core/2D/MeshingContext2D.h"
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace Geometry3D
{
class ISurface3D;
class GeometryCollection3D;
} // namespace Geometry3D

namespace Topology3D
{
class Surface3D;
class Topology3D;
} // namespace Topology3D

namespace Meshing
{

/**
 * @brief Manages 2D Delaunay triangulation in parametric space for a single 3D surface
 *
 * Each facet (surface) maintains its own independent 2D triangulation that is
 * updated when vertices are inserted on the facet during refinement. This class
 * maintains bidirectional mapping between 3D mesh node IDs and 2D facet node IDs.
 *
 * Per Shewchuk's algorithm, facet triangulations define what subfacets *should*
 * exist in the final mesh. They are maintained separately from the 3D tetrahedralization.
 */
class FacetTriangulation
{
public:
    /**
     * @brief Construct facet triangulation for a surface
     * @param surface The 3D surface geometry
     * @param topoSurface The surface topology
     * @param fullTopology Full 3D topology for edge/corner lookups
     * @param fullGeometry Full 3D geometry collection
     */
    FacetTriangulation(const Geometry3D::ISurface3D& surface,
                       const Topology3D::Surface3D& topoSurface,
                       const Topology3D::Topology3D& fullTopology,
                       const Geometry3D::GeometryCollection3D& fullGeometry);

    ~FacetTriangulation();

    // Prevent copying
    FacetTriangulation(const FacetTriangulation&) = delete;
    FacetTriangulation& operator=(const FacetTriangulation&) = delete;

    // Allow moving
    FacetTriangulation(FacetTriangulation&&) noexcept;
    FacetTriangulation& operator=(FacetTriangulation&&) noexcept;

    /**
     * @brief Initialize triangulation with boundary and interior points
     *
     * Creates a 2D Delaunay triangulation of the given points. The points
     * are provided with their 3D node IDs and corresponding (u,v) parametric
     * coordinates on this surface.
     *
     * @param node3DToPoint2DMap Maps 3D node IDs to their (u,v) coordinates on this surface
     */
    void initialize(const std::map<size_t, Point2D>& node3DToPoint2DMap);

    /**
     * @brief Get the surface ID
     */
    const std::string& getSurfaceId() const { return surfaceId_; }

    /**
     * @brief Get all subfacets (triangles) from the 2D triangulation
     *
     * Converts each 2D triangle to a ConstrainedSubfacet3D using the
     * 2D->3D node ID mapping.
     *
     * @return Vector of ConstrainedSubfacet3D with 3D node IDs
     */
    std::vector<ConstrainedSubfacet3D> getSubfacets() const;

    /**
     * @brief Insert a vertex into the facet triangulation
     *
     * Used during refinement when splitting subfacets. The vertex is
     * inserted into the 2D triangulation and the mappings are updated.
     *
     * @param node3DId The 3D mesh node ID
     * @param uvCoords The (u,v) parametric coordinates
     * @return true if insertion succeeded
     */
    bool insertVertex(size_t node3DId, const Point2D& uvCoords);

    /**
     * @brief Get the 2D node ID corresponding to a 3D node ID
     * @param node3DId The 3D mesh node ID
     * @return The 2D facet node ID, or std::nullopt if not found
     */
    std::optional<size_t> get2DNodeId(size_t node3DId) const;

    /**
     * @brief Get the 3D node ID corresponding to a 2D node ID
     * @param node2DId The 2D facet node ID
     * @return The 3D mesh node ID, or std::nullopt if not found
     */
    std::optional<size_t> get3DNodeId(size_t node2DId) const;

    /**
     * @brief Access the underlying 2D mesh context
     */
    MeshingContext2D& getContext() { return *context_; }
    const MeshingContext2D& getContext() const { return *context_; }

private:
    std::string surfaceId_;
    const Geometry3D::ISurface3D* surface_;

    std::unique_ptr<MeshingContext2D> context_;

    // Bidirectional mapping between 3D mesh node IDs and 2D facet node IDs
    std::map<size_t, size_t> node3DTo2DMap_;
    std::map<size_t, size_t> node2DTo3DMap_;
};

} // namespace Meshing
