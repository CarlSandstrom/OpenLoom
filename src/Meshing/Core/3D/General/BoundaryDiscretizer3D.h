#pragma once

#include "Meshing/Core/3D/General/DiscretizationResult3D.h"
#include "Geometry/3D/Base/DiscretizationSettings3D.h"

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
 * discretization settings. The result can be used directly by Delaunay3D
 * for tetrahedralization, and the mappings facilitate constraint setup.
 *
 * Example usage:
 * @code
 * BoundaryDiscretizer3D discretizer(geometry, topology, settings);
 * auto discretization = discretizer.discretize();
 * @endcode
 */
class BoundaryDiscretizer3D
{
public:
    /**
     * @brief Construct a boundary discretizer
     * @param geometry The geometry collection containing corners, edges, and surfaces
     * @param topology The topology defining connectivity between geometric entities
     * @param settings Discretization settings (segments per edge, surface samples)
     */
    BoundaryDiscretizer3D(const Geometry3D::GeometryCollection3D& geometry,
                          const Topology3D::Topology3D& topology,
                          const Geometry3D::DiscretizationSettings3D& settings = {});

    /**
     * @brief Discretize all boundaries in the geometry
     * @return Discretization result containing sampled points and mappings
     */
    DiscretizationResult3D discretize() const;

private:
    const Geometry3D::GeometryCollection3D* geometry_;
    const Topology3D::Topology3D* topology_;
    Geometry3D::DiscretizationSettings3D settings_;
};

} // namespace Meshing
