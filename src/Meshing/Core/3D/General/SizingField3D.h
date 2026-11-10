#pragma once

#include "Common/Types.h"

#include <vector>

namespace Meshing
{

/// One "elements should be about this large, here" observation gathered from
/// the geometry by SizingFieldBuilder3D. The gradient limit decides how quickly
/// its constraint relaxes with distance.
struct SizingSource
{
    Point3D position;
    double size = 0.0;
};

/**
 * @brief The mesh sizing field h(x): how large an element should be at a
 * given point in space (OPE-181).
 *
 * Quality criteria cannot play this role: the angle and circumradius /
 * shortest-edge bounds are shape bounds, scale-invariant by construction, so a
 * well-shaped element of any size satisfies them.
 *
 * ## A gradient-limited envelope over point sources
 *
 * Given sources (x_i, h_i) and gradient limit g, the field is
 *
 *     h(x) = min over i of ( h_i + g * |x - x_i| )
 *
 * Each term is g-Lipschitz, and so is their minimum, so |grad h| <= g holds
 * everywhere by construction. Among all such functions bounded by the sources
 * this is the largest, meaning the least over-refined. Gradient limiting
 * therefore needs no background grid and no relaxation sweep. g is the maximum
 * fractional change in element size per unit length; the usual range is
 * 0.2-0.5, and smaller means smoother but more elements.
 *
 * ## Two consequences to know before adding sources
 *
 * The distance is ambient, not geodesic along the surface, so two sheets that
 * are close through space limit each other. That is conservative (it only asks
 * for smaller elements) and is the desired behaviour for local-feature-size
 * sources, since the mesh has to resolve the gap between them.
 *
 * A missing source is harmless: its neighbours still cover the position
 * through the envelope, merely less tightly. A wrong, too-small source drags
 * down a whole neighbourhood. Where the geometry cannot be evaluated reliably
 * (a parametric pole, a cone apex, a degenerate patch) emit no source rather
 * than guess one; see ISurface3D::getPrincipalCurvatures().
 */
class SizingField3D
{
public:
    /// Throws if sources is empty or any size or gradientLimit is not finite
    /// and strictly positive: these are programming errors in the builder.
    SizingField3D(std::vector<SizingSource> sources, double gradientLimit);

    /// The desired element size at point. O(number of sources), which is set by
    /// the geometry (hundreds to a few thousand) rather than by the mesh. If it
    /// becomes hot, a spatial index belongs behind this call.
    double evaluate(const Point3D& point) const;

    double getGradientLimit() const { return gradientLimit_; }

    const std::vector<SizingSource>& getSources() const { return sources_; }

    /// The smallest size any source asks for: the finest the field ever gets,
    /// reached only at that source's own position.
    double getMinimumSourceSize() const { return minimumSourceSize_; }

private:
    std::vector<SizingSource> sources_;
    double gradientLimit_;
    double minimumSourceSize_;
};

} // namespace Meshing
