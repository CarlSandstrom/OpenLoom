#pragma once

#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/General/DiscretizationResult3D.h"
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
 * discretization settings.
 *
 * The result is owned by the caller via the release method so it can outlive
 * the discretizer:
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
     */
    BoundaryDiscretizer3D(const Geometry3D::GeometryCollection3D& geometry,
                          const Topology3D::Topology3D& topology,
                          const Geometry3D::DiscretizationSettings3D& settings = {});

    /**
     * @brief Discretize all boundaries in the geometry.
     *
     * Populates the internal DiscretizationResult3D.
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

private:
    const Geometry3D::GeometryCollection3D* geometry_;
    const Topology3D::Topology3D* topology_;
    Geometry3D::DiscretizationSettings3D settings_;

    std::unique_ptr<DiscretizationResult3D> result_;
};

} // namespace Meshing
