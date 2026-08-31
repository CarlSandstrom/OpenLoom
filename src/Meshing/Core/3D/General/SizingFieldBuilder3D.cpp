#include "Meshing/Core/3D/General/SizingFieldBuilder3D.h"

#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/IEdge3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Core/3D/General/LocalFeatureSize3D.h"
#include "Topology/Topology3D.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace Meshing
{

namespace
{

/// The element size at which a chord across curvature kappa stands off the
/// true geometry by exactly tolerance. Infinite where the geometry is
/// straight or flat, which correctly means "this imposes no size limit"
/// rather than "zero".
double sizeFromCurvature(double curvature, double tolerance)
{
    if (!(curvature > 0.0) || !std::isfinite(curvature))
        return std::numeric_limits<double>::infinity();

    return std::sqrt(8.0 * tolerance / curvature);
}

double boundingBoxDiagonal(const std::vector<FeatureSample>& samples)
{
    if (samples.empty())
        return 0.0;

    Point3D minimum = samples.front().position;
    Point3D maximum = samples.front().position;

    for (const auto& sample : samples)
    {
        minimum = minimum.cwiseMin(sample.position);
        maximum = maximum.cwiseMax(sample.position);
    }

    return (maximum - minimum).norm();
}

} // namespace

SizingField3D SizingFieldBuilder3D::build(const Geometry3D::GeometryCollection3D& geometry,
                                          const Topology3D::Topology3D& topology,
                                          const SizingFieldSettings3D& settings)
{
    std::vector<FeatureSample> samples;
    std::vector<double> curvatureSizes;

    // Curves. A seam twin duplicates its original's points in reverse (see
    // BoundaryDiscretizer3D) and would only contribute the same samples
    // twice; a degenerate edge has no real 3D curve to probe at all.
    for (const auto& edgeId : topology.getAllEdgeIds())
    {
        if (topology.getSeamCollection().isSeamTwin(edgeId))
            continue;

        const auto* edge = geometry.getEdge(edgeId);
        if (!edge || edge->isDegenerate())
            continue;

        const auto [tMinimum, tMaximum] = edge->getParameterBounds();
        const std::size_t stations = std::max<std::size_t>(settings.samplesPerCurve, 2);

        double arcLength = 0.0;
        Point3D previous = edge->getPoint(tMinimum);

        for (std::size_t station = 0; station <= stations; ++station)
        {
            const double t = tMinimum + (tMaximum - tMinimum) * static_cast<double>(station) /
                                            static_cast<double>(stations);
            const Point3D position = edge->getPoint(t);
            arcLength += (position - previous).norm();
            previous = position;

            samples.push_back({position, edgeId, arcLength, true});
            curvatureSizes.push_back(
                sizeFromCurvature(edge->getCurvature(t), settings.chordDeviationTolerance));
        }
    }

    // Surfaces.
    for (const auto& surfaceId : topology.getAllSurfaceIds())
    {
        const auto* surface = geometry.getSurface(surfaceId);
        if (!surface)
            continue;

        const auto bounds = surface->getParameterBounds();
        const std::size_t stations = std::max<std::size_t>(settings.samplesPerSurfaceDirection, 2);

        for (std::size_t i = 0; i <= stations; ++i)
        {
            const double u = bounds.getUMin() + (bounds.getUMax() - bounds.getUMin()) *
                                                    static_cast<double>(i) /
                                                    static_cast<double>(stations);

            for (std::size_t j = 0; j <= stations; ++j)
            {
                const double v = bounds.getVMin() + (bounds.getVMax() - bounds.getVMin()) *
                                                        static_cast<double>(j) /
                                                        static_cast<double>(stations);

                // A uniform grid over the untrimmed parameter rectangle can
                // land in a hole or cutout (see OPE-169); such a point is
                // not on the model and must not seed the field.
                if (!surface->isUVWithinTrimmedBoundary(u, v))
                    continue;

                const auto curvatures = surface->getPrincipalCurvatures(u, v);
                const double curvature =
                    curvatures.has_value() ? std::max(std::abs(curvatures->minimum), std::abs(curvatures->maximum)) : 0.0;

                samples.push_back({surface->getPoint(u, v), surfaceId, 0.0, false});
                curvatureSizes.push_back(
                    sizeFromCurvature(curvature, settings.chordDeviationTolerance));
            }
        }
    }

    const std::vector<double> featureSizes =
        LocalFeatureSize3D::compute(samples, topology);

    const double diagonal = boundingBoxDiagonal(samples);
    const double floor = settings.minimumSizeFraction * diagonal;

    std::vector<SizingSource> sources;
    sources.reserve(samples.size());

    for (std::size_t i = 0; i < samples.size(); ++i)
    {
        const double size =
            std::min(curvatureSizes[i], featureSizes[i] / settings.elementsAcrossFeatureGap);

        // Neither term constrains this sample -- flat geometry with nothing
        // unrelated nearby. Emitting no source is the correct answer; the
        // envelope covers the position from its neighbours instead.
        if (!std::isfinite(size))
            continue;

        sources.push_back({samples[i].position, std::max(size, floor)});
    }

    if (sources.empty())
    {
        // No curvature and no proximity anywhere: the geometry genuinely
        // imposes no size limit. One coarse source at model scale says
        // exactly that, and keeps every consumer on the same code path.
        // Sizing a source too LARGE is safe; too small is not.
        sources.push_back({Point3D::Zero(), diagonal > 0.0 ? diagonal : 1.0});
    }

    return SizingField3D(std::move(sources), settings.gradientLimit);
}

} // namespace Meshing
