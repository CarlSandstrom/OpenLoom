#pragma once

#include "Common/TwinManager.h"
#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/General/DiscretizationResult3D.h"
#include "Meshing/Core/3D/General/EdgeTwinTable.h"
#include <memory>

namespace Geometry3D
{
class GeometryCollection3D;
}

namespace Topology3D
{
class Topology3D;
}

namespace Meshing
{

/**
 * @brief Discretizes 3D geometry boundaries into points for mesh generation
 *
 * Samples points along corners, edges, and surface interiors based on
 * discretization settings. When an EdgeTwinTable is supplied, also builds
 * a TwinManager that maps shared-edge segment pairs for inter-face conformity.
 *
 * Both results are owned by the caller via the release methods so they can
 * outlive the discretizer:
 * @code
 * auto twinTable = TwinTableGenerator::generate(topology);
 * BoundaryDiscretizer3D disc(geometry, topology, settings, twinTable);
 * disc.discretize();
 * auto result = disc.releaseDiscretizationResult(); // caller takes ownership
 * auto twins  = disc.releaseTwinManager();          // caller takes ownership
 * @endcode
 *
 * Example usage (volume mesher — no twins):
 * @code
 * BoundaryDiscretizer3D disc(geometry, topology, settings);
 * disc.discretize();
 * auto result = disc.releaseDiscretizationResult();
 * @endcode
 */
class BoundaryDiscretizer3D
{
public:
    /**
     * @brief Construct a boundary discretizer
     * @param geometry  The geometry collection containing corners, edges, and surfaces
     * @param topology  The topology defining connectivity between geometric entities
     * @param settings  Discretization settings (segments per edge, surface samples)
     * @param twinTable Shared-edge table for twin-segment registration (default: empty)
     */
    BoundaryDiscretizer3D(const Geometry3D::GeometryCollection3D& geometry,
                          const Topology3D::Topology3D& topology,
                          const Geometry3D::DiscretizationSettings3D& settings = {},
                          const EdgeTwinTable& twinTable = {});

    /**
     * @brief Discretize all boundaries in the geometry.
     *
     * Populates the internal DiscretizationResult3D and, if a non-empty
     * EdgeTwinTable was provided, the internal TwinManager.
     * Safe to call multiple times — subsequent calls recompute from scratch.
     */
    void discretize();

    /**
     * @brief Non-owning view of the discretization result.
     *
     * Valid until releaseDiscretizationResult() is called or the discretizer
     * is destroyed. After release, must not be called.
     */
    const DiscretizationResult3D& getDiscretizationResult() const;

    /**
     * @brief Transfer ownership of the discretization result to the caller.
     *
     * After this call getDiscretizationResult() must not be called.
     */
    std::unique_ptr<DiscretizationResult3D> releaseDiscretizationResult();

    /**
     * @brief Non-owning view of the TwinManager built by discretize().
     *
     * Valid until releaseTwinManager() is called or the discretizer is destroyed.
     * Empty if no EdgeTwinTable was provided or discretize() has not been called.
     */
    const TwinManager& getTwinManager() const;

    /**
     * @brief Transfer ownership of the TwinManager to the caller.
     *
     * The TwinManager must outlive the refiner and BoundarySplitSynchronizer
     * that reference it, so the caller (e.g. SurfaceMeshingContext3D) should
     * hold the returned unique_ptr for the full meshing lifetime.
     *
     * After this call getTwinManager() must not be called.
     */
    std::unique_ptr<TwinManager> releaseTwinManager();

private:
    const Geometry3D::GeometryCollection3D* geometry_;
    const Topology3D::Topology3D* topology_;
    Geometry3D::DiscretizationSettings3D settings_;
    EdgeTwinTable twinTable_;

    std::unique_ptr<DiscretizationResult3D> result_;
    std::unique_ptr<TwinManager> twinManager_;
};

} // namespace Meshing
