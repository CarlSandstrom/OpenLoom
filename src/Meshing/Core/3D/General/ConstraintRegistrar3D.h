#pragma once

#include "Meshing/Core/3D/General/DiscretizationResult3D.h"
#include "Meshing/Core/3D/General/FacetTriangulationManager.h"
#include <map>
#include <memory>

namespace Meshing
{

class MeshingContext3D;

/**
 * @brief Registers boundary constraints (subsegments and subfacets) from discretization
 *
 * Per Shewchuk's 3D Delaunay refinement algorithm, constraints are stored
 * separately from the tetrahedralization. The initial Delaunay mesh may not
 * contain all constraint edges/faces - the refinement process recovers them
 * by splitting encroached subsegments and subfacets.
 *
 * This class extracts constraints from the discretization result and registers
 * them in MeshData3D. It also manages the FacetTriangulationManager for tracking
 * the 2D triangulations of each facet.
 *
 * Usage:
 * @code
 *   // 1. Discretize boundaries
 *   BoundaryDiscretizer3D discretizer(context, settings);
 *   auto discretization = discretizer.discretize();
 *
 *   // 2. Create unconstrained Delaunay tetrahedralization
 *   Delaunay3D delaunay(discretization.points, &context.getMeshData(), ...);
 *   delaunay.triangulate();
 *
 *   // 3. Register constraints (may not appear in mesh yet)
 *   ConstraintRegistrar3D registrar(context, discretization);
 *   registrar.registerConstraints(delaunay.getPointIndexToNodeIdMap());
 *
 *   // 4. Refine - recovers missing constraints + improves quality
 *   ShewchukRefiner3D refiner(context, qualityController);
 *   refiner.refine();
 * @endcode
 */
class ConstraintRegistrar3D
{
public:
    /**
     * @brief Construct registrar with context and discretization
     * @param context The meshing context (provides geometry, topology, mesh data)
     * @param discretization The boundary discretization result
     */
    ConstraintRegistrar3D(MeshingContext3D& context,
                          const DiscretizationResult3D& discretization);

    ~ConstraintRegistrar3D();

    // Prevent copying
    ConstraintRegistrar3D(const ConstraintRegistrar3D&) = delete;
    ConstraintRegistrar3D& operator=(const ConstraintRegistrar3D&) = delete;

    // Allow moving
    ConstraintRegistrar3D(ConstraintRegistrar3D&&) noexcept;
    ConstraintRegistrar3D& operator=(ConstraintRegistrar3D&&) noexcept;

    /**
     * @brief Register all constraints from discretization
     *
     * Extracts subsegments from topology edges and subfacets from topology
     * surfaces, then stores them in MeshData3D. The constraints define what
     * edges and faces *should* appear in the final mesh.
     *
     * @param pointIndexToNodeIdMap Maps discretization point indices to mesh node IDs
     *                               (obtained from Delaunay3D::getPointIndexToNodeIdMap())
     */
    void registerConstraints(const std::map<size_t, size_t>& pointIndexToNodeIdMap);

    /**
     * @brief Get the facet triangulation manager
     *
     * Returns the manager that tracks 2D triangulations for each facet.
     * The refiner needs this to update facet triangulations when inserting
     * vertices and to check for missing subfacets.
     *
     * @return Pointer to the FacetTriangulationManager, or nullptr if not yet created
     */
    FacetTriangulationManager* getFacetManager() { return facetManager_.get(); }
    const FacetTriangulationManager* getFacetManager() const { return facetManager_.get(); }

    /**
     * @brief Transfer ownership of the facet manager
     *
     * Used when the refiner needs to take ownership of the facet manager
     * for updating during refinement.
     *
     * @return Unique pointer to the FacetTriangulationManager
     */
    std::unique_ptr<FacetTriangulationManager> releaseFacetManager();

private:
    MeshingContext3D* context_;
    const DiscretizationResult3D* discretization_;
    std::map<size_t, size_t> pointIndexToNodeIdMap_;
    std::unique_ptr<FacetTriangulationManager> facetManager_;

    /**
     * @brief Extract and store subsegments for all topology edges
     */
    void registerSubsegments();

    /**
     * @brief Create facet triangulations and store subfacets
     */
    void registerSubfacets();
};

} // namespace Meshing
