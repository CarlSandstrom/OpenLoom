#pragma once

#include "DiscretizationResult3D.h"
#include "Geometry/3D/Base/DiscretizationSettings3D.h"

namespace Meshing
{

class MeshingContext3D;

/**
 * @brief Discretizes 3D geometry boundaries into points for mesh generation
 *
 * Samples points along corners, edges, and surface interiors based on
 * discretization settings. The result can be used directly by Delaunay3D
 * for tetrahedralization, and the mappings facilitate constraint setup.
 *
 * Example usage:
 * @code
 * MeshingContext3D context(geometry, topology);
 * BoundaryDiscretizer3D discretizer(context, settings);
 * auto discretization = discretizer.discretize();
 * Delaunay3D delaunay(discretization.points, &context.getMeshData(), ...);
 * @endcode
 */
class BoundaryDiscretizer3D
{
public:
    /**
     * @brief Construct a boundary discretizer
     * @param context The meshing context containing geometry and topology
     * @param settings Discretization settings (segments per edge, surface samples)
     */
    BoundaryDiscretizer3D(const MeshingContext3D& context,
                          const Geometry3D::DiscretizationSettings3D& settings = {});

    /**
     * @brief Discretize all boundaries in the geometry
     * @return Discretization result containing sampled points and mappings
     */
    DiscretizationResult3D discretize() const;

private:
    const MeshingContext3D* context_;
    Geometry3D::DiscretizationSettings3D settings_;
};

} // namespace Meshing
