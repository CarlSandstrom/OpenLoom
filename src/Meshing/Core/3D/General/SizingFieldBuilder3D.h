#pragma once

#include "Meshing/Core/3D/General/SizingField3D.h"

#include <cstddef>

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

/// Inputs controlling how SizingFieldBuilder3D turns CAD geometry into h(x).
struct SizingFieldSettings3D
{
    /// Maximum permitted distance between a straight element and the curved
    /// geometry it approximates, in model units. This is the same quantity
    /// as SurfaceMesh3DQualitySettings::chordDeviationTolerance, and the two
    /// should agree -- the sizing field exists so that refinement's size
    /// target and the geometry's own demands are one number rather than two.
    double chordDeviationTolerance = 0.1;

    /// Maximum fractional change in element size per unit length. Smaller
    /// grades more smoothly and costs more elements; 0.2-0.5 is the usual
    /// range across meshers.
    double gradientLimit = 0.3;

    /// How many elements to place across a gap between unrelated pieces of
    /// geometry. Two is the minimum that resolves a gap at all rather than
    /// bridging it.
    double elementsAcrossFeatureGap = 2.0;

    /// Stations per curve and per surface parameter direction used to probe
    /// the geometry. Higher costs only build time, and only linearly (the
    /// local-feature-size pass is quadratic in the resulting sample count).
    /// Too low risks stepping over a short high-curvature stretch entirely.
    std::size_t samplesPerCurve = 32;
    std::size_t samplesPerSurfaceDirection = 8;

    /// Absolute floor on any size the field will report, as a fraction of
    /// the model's bounding-box diagonal. Guards against a curvature
    /// singularity (a fillet collapsing to a point, a cusp) demanding an
    /// unbounded number of elements.
    double minimumSizeFraction = 1e-3;
};

/**
 * @brief Builds the mesh sizing field h(x) from CAD geometry (OPE-181).
 *
 * Combines the two independent things that force elements to be small:
 *
 *  - **Curvature**, inverted through the chord-deviation tolerance. A chord
 *    of length h across a curve of radius r = 1/kappa stands off it by about
 *    h^2 * kappa / 8, so holding that to `tolerance` means
 *    h = sqrt(8 * tolerance / kappa). Curves use IEdge3D::getCurvature();
 *    surfaces use the larger principal curvature magnitude from
 *    ISurface3D::getPrincipalCurvatures().
 *  - **Local feature size**, from LocalFeatureSize3D -- how close unrelated
 *    geometry is, divided by how many elements should span the gap.
 *
 * The smaller of the two wins at each sample, and the resulting sources are
 * handed to SizingField3D, whose envelope does the gradient limiting. Note
 * that the field is therefore only as good as its sampling: a high-curvature
 * stretch shorter than the station spacing can be stepped over entirely.
 * Sampling densely is cheap here (see samplesPerCurve) precisely because the
 * result is a compact set of sources rather than a mesh.
 *
 * Where the geometry cannot be evaluated -- a degenerate edge, a parametric
 * pole, a point outside a trimmed patch -- no source is emitted rather than
 * a guessed one. See SizingField3D's class comment for why that asymmetry is
 * safe in one direction and not the other.
 */
class SizingFieldBuilder3D
{
public:
    static SizingField3D build(const Geometry3D::GeometryCollection3D& geometry,
                               const Topology3D::Topology3D& topology,
                               const SizingFieldSettings3D& settings = {});
};

} // namespace Meshing
