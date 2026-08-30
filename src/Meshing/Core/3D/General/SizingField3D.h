#pragma once

#include "Common/Types.h"

#include <vector>

namespace Meshing
{

/// One "elements should be about this large, here" observation, as gathered
/// from the geometry by SizingFieldBuilder3D. A source constrains the field
/// at its own position; the field's gradient limit decides how quickly that
/// constraint is allowed to relax with distance.
struct SizingSource
{
    Point3D position;
    double size = 0.0;
};

/**
 * @brief The mesh sizing field h(x): how large an element should be at a
 * given point in space (OPE-181).
 *
 * RCDT historically had no such object. Element size emerged instead from
 * three unrelated criteria in three different units -- an angle
 * (BoundaryDiscretizer3D's maxAngleBetweenSegments), a distance
 * (RCDTQualityController's chordDeviationTolerance), and a segment-length
 * multiple (CurveProtectionScheme's protecting-ball radii) -- which had no
 * way to agree with one another. This class is the single explicit answer
 * every one of those consumers is meant to read instead.
 *
 * Note that no quality criterion can play this role.
 * circumradiusToShortestEdgeRatio and friends are SHAPE bounds, scale-
 * invariant by construction, so a perfectly-shaped element of any size at
 * all satisfies them. Controlling size requires a size field.
 *
 * ## The field is a gradient-limited envelope over point sources
 *
 * Given sources (x_i, h_i), the field is
 *
 *     h(x) = min over i of ( h_i + g * |x - x_i| )
 *
 * for gradient limit g. This is not an approximation of a gradient-limited
 * field -- it IS one, exactly. Each term is g-Lipschitz, a minimum of
 * g-Lipschitz functions is g-Lipschitz, so |grad h| <= g holds everywhere by
 * construction, and among all such functions bounded by the sources this is
 * the largest (i.e. the least over-refined). Gradient limiting therefore
 * needs no background grid, no fast-marching or relaxation sweep, and no
 * resolution parameter -- all of which a Persson-style limiter on a general
 * field would require.
 *
 * Gradient limiting matters because ungraded size fields produce badly
 * shaped elements where a fine region meets a coarse one. g is the maximum
 * fractional change in element size per unit length; the usual range is
 * 0.2-0.5, and smaller means smoother but more elements.
 *
 * ## Two consequences worth knowing before adding sources
 *
 * The distance above is AMBIENT, not geodesic along the surface. Two sheets
 * that are far apart across the surface but close through space limit each
 * other. That is conservative -- it can only ask for smaller elements, never
 * larger -- and for the local-feature-size sources it is exactly the desired
 * behaviour, since the mesh does have to resolve the gap between them.
 *
 * A MISSING source is harmless: neighbouring sources still cover its
 * position through the envelope, merely less tightly. A WRONG, too-small
 * source is not -- it drags down a whole neighbourhood of the field. So
 * wherever the geometry cannot be evaluated reliably (a parametric pole, a
 * cone apex, a degenerate patch), the correct response is to emit no source
 * there rather than to guess one. See ISurface3D::getPrincipalCurvatures(),
 * which reports that case as nullopt rather than inventing a value.
 */
class SizingField3D
{
public:
    /// sources must be non-empty and every size strictly positive;
    /// gradientLimit must be strictly positive. All three are programming
    /// errors on the builder's part rather than recoverable conditions, so
    /// they throw.
    SizingField3D(std::vector<SizingSource> sources, double gradientLimit);

    /// The desired element size at point. O(number of sources) -- see the
    /// class comment; this is deliberate, since the source count is set by
    /// the geometry (hundreds to a few thousand) rather than by the mesh.
    /// If it ever becomes hot, a spatial index belongs behind this call, not
    /// in its callers.
    double evaluate(const Point3D& point) const;

    double getGradientLimit() const { return gradientLimit_; }

    const std::vector<SizingSource>& getSources() const { return sources_; }

    /// The smallest size any source asks for -- i.e. the finest the field
    /// can ever be, reached only at that source's own position. Callers
    /// deriving a floor (RCDTRefiner's minimumEdgeLength) want this rather
    /// than a sampled minimum.
    double getMinimumSourceSize() const { return minimumSourceSize_; }

private:
    std::vector<SizingSource> sources_;
    double gradientLimit_;
    double minimumSourceSize_;
};

} // namespace Meshing
