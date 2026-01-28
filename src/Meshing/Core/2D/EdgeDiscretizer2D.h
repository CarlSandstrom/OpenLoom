#pragma once

#include "DiscretizationResult2D.h"
#include "Geometry/2D/Base/DiscretizationSettings2D.h"

namespace Meshing
{

class MeshingContext2D;

/**
 * @brief Discretizes 2D geometry edges into points for mesh generation
 *
 * Samples points along curved edges based on discretization settings
 * (curvature-based or fixed segment count). The result can be used
 * directly by ConstrainedDelaunay2D for triangulation.
 *
 * Example usage:
 * @code
 * MeshingContext2D context(geometry, topology);
 * EdgeDiscretizer2D discretizer(context, settings);
 * auto discretization = discretizer.discretize();
 * ConstrainedDelaunay2D mesher(context, discretization);
 * @endcode
 */
class EdgeDiscretizer2D
{
public:
    /**
     * @brief Construct an edge discretizer
     * @param context The meshing context containing geometry and topology
     * @param settings Discretization settings (curvature tolerance, segment count)
     */
    EdgeDiscretizer2D(const MeshingContext2D& context,
                      const Geometry2D::DiscretizationSettings2D& settings = {});

    /**
     * @brief Discretize all edges in the geometry
     * @return Discretization result containing sampled points and mappings
     */
    DiscretizationResult2D discretize() const;

private:
    const MeshingContext2D* context_;
    Geometry2D::DiscretizationSettings2D settings_;
};

} // namespace Meshing
