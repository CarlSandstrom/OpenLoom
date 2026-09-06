#pragma once

#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/General/DiscretizationResult3D.h"
#include "Meshing/Core/3D/General/SizingField3D.h"
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
 *
 * ## Bounding segment length with a sizing field (OPE-181, opt-in)
 *
 * DiscretizationSettings3D's angle criterion is scale-free: it bounds how far
 * a segment may TURN, never how long it may be. Where a curve is steep but
 * barely turning, that permits an arbitrarily long segment -- on
 * SaddleSurfaceMesh's parabolic top arc it produces chords of 3.79 next to
 * chords of 0.20, an 18.9x spread on one curve. CurveProtectionScheme then
 * sizes protecting balls as a multiple of the LONGER adjacent segment, so
 * that spread becomes a ball wide enough to swallow its own neighbours,
 * hiding the finely-sampled crease points entirely.
 *
 * Passing a sizing field adds the missing length bound: a point is
 * emitted as soon as EITHER the tangent has turned by maxAngleBetweenSegments
 * OR the accumulated arclength has reached h(x). Because SizingField3D is
 * Lipschitz by construction, adjacent segments cannot then differ by more
 * than the gradient limit allows.
 *
 * The field is supplied rather than built here: h(x) is a property of the
 * whole model, and the floor RCDTMesher derives for minimumEdgeLength has to
 * read the SAME field this walk did. Building one privately per consumer is
 * the very decoupling OPE-181 exists to remove.
 *
 * This is opt-in and off by default: with no field the walk is
 * byte-identical to the angle-only behaviour above.
 */
class BoundaryDiscretizer3D
{
public:
    /**
     * @brief Construct a boundary discretizer
     * @param geometry  The geometry collection containing corners, edges, and surfaces
     * @param topology  The topology defining connectivity between geometric entities
     * @param settings  Discretization settings (segments per edge, surface samples)
     * @param sizingField  When non-null, bounds edge segment length by h(x)
     *        as well as by tangent angle (see the class comment). Null leaves
     *        segment length unbounded, which is the historical behaviour.
     *        Only affects the angle-based mode; fixed-count mode is defined
     *        as uniform subdivision and is left alone. Borrowed, not owned --
     *        must outlive discretize().
     */
    BoundaryDiscretizer3D(const Geometry3D::GeometryCollection3D& geometry,
                          const Topology3D::Topology3D& topology,
                          const Geometry3D::DiscretizationSettings3D& settings = {},
                          const SizingField3D* sizingField = nullptr);

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
    const SizingField3D* sizingField_;

    std::unique_ptr<DiscretizationResult3D> result_;
};

} // namespace Meshing
