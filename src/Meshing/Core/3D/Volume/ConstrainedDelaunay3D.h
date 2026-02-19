#pragma once

#include "Common/Types.h"
#include "Meshing/Core/3D/General/DiscretizationResult3D.h"
#include "Meshing/Core/3D/Surface/FacetTriangulationManager.h"
#include <map>
#include <memory>
#include <vector>

namespace Meshing
{

class MeshingContext3D;
class MeshData3D;
class MeshOperations3D;

/**
 * @brief Orchestrates 3D constrained Delaunay tetrahedralization
 *
 * Implements the input processing phase (Step 1) of Shewchuk's 3D Delaunay
 * refinement algorithm:
 *
 * 1. Create initial unconstrained Delaunay tetrahedralization from discretized points
 * 2. Extract constrained subsegments from topology edges
 * 3. Create facet triangulations for all surfaces (independent 2D Delaunay)
 * 4. Store subfacets as constraints in MeshData3D
 *
 * Note: The tetrahedralization is NOT constrained. Constraints (subsegments,
 * subfacets) are tracked but may not exist in the mesh. They will be recovered
 * through refinement in later algorithm phases.
 */
class ConstrainedDelaunay3D
{
public:
    /**
     * @brief Construct 3D Delaunay triangulator with MeshingContext3D
     * @param context The 3D meshing context containing geometry and topology
     * @param discretization Pre-computed boundary discretization
     */
    explicit ConstrainedDelaunay3D(MeshingContext3D& context,
                                    const DiscretizationResult3D& discretization);

    ~ConstrainedDelaunay3D();

    // Prevent copying
    ConstrainedDelaunay3D(const ConstrainedDelaunay3D&) = delete;
    ConstrainedDelaunay3D& operator=(const ConstrainedDelaunay3D&) = delete;

    // Allow moving
    ConstrainedDelaunay3D(ConstrainedDelaunay3D&&) noexcept;
    ConstrainedDelaunay3D& operator=(ConstrainedDelaunay3D&&) noexcept;

    /**
     * @brief Perform the initial tetrahedralization and constraint setup
     *
     * Steps performed:
     * 1. Create initial Delaunay tetrahedralization from discretization points
     * 2. Extract constrained subsegments from topology edges
     * 3. Create facet triangulations for all surfaces
     * 4. Store subsegments and subfacets as constraints in MeshData3D
     */
    void tetrahedralize();

    /**
     * @brief Access the facet triangulation manager
     *
     * Needed for refinement steps that insert vertices on surfaces.
     */
    FacetTriangulationManager& getFacetManager() { return *facetManager_; }
    const FacetTriangulationManager& getFacetManager() const { return *facetManager_; }

    /**
     * @brief Get the point index to node ID mapping from the initial triangulation
     */
    const std::map<size_t, size_t>& getPointIndexToNodeIdMap() const { return pointIndexToNodeIdMap_; }

private:
    DiscretizationResult3D discretization_;

    MeshingContext3D* context_;
    MeshData3D* meshData3D_;
    MeshOperations3D* meshOperations_;

    size_t exportCounter_ = 0;

    std::unique_ptr<FacetTriangulationManager> facetManager_;
    std::map<size_t, size_t> pointIndexToNodeIdMap_;

    void extractAndStoreSubsegments();
    void createFacetTriangulations();
    void storeSubfacets();
};

} // namespace Meshing
