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

struct GeometrySamples
{
    std::vector<FeatureSample> samples;
    std::vector<double> curvatureSizes;
};

double sizeFromCurvature(double curvature, double tolerance);
double stationParameter(double minimum, double maximum, std::size_t station, std::size_t stationCount);
double boundingBoxDiagonal(const std::vector<FeatureSample>& samples);
void appendCurveSamples(const Geometry3D::GeometryCollection3D& geometry,
                        const Topology3D::Topology3D& topology,
                        const SizingFieldSettings3D& settings,
                        GeometrySamples& result);
void appendSurfaceSamples(const Geometry3D::GeometryCollection3D& geometry,
                          const Topology3D::Topology3D& topology,
                          const SizingFieldSettings3D& settings,
                          GeometrySamples& result);
std::vector<SizingSource> buildSources(const GeometrySamples& geometrySamples,
                                       const std::vector<double>& featureSizes,
                                       const SizingFieldSettings3D& settings);

} // namespace

SizingField3D SizingFieldBuilder3D::build(const Geometry3D::GeometryCollection3D& geometry,
                                          const Topology3D::Topology3D& topology,
                                          const SizingFieldSettings3D& settings)
{
    GeometrySamples geometrySamples;
    appendCurveSamples(geometry, topology, settings, geometrySamples);
    appendSurfaceSamples(geometry, topology, settings, geometrySamples);

    auto featureSizes = LocalFeatureSize3D::compute(geometrySamples.samples, topology);

    return SizingField3D(buildSources(geometrySamples, featureSizes, settings), settings.gradientLimit);
}

namespace
{

// Infinite where the geometry is straight or flat, which means "no size limit", not "zero".
double sizeFromCurvature(double curvature, double tolerance)
{
    if (!(curvature > 0.0) || !std::isfinite(curvature)) return std::numeric_limits<double>::infinity();

    return std::sqrt(8.0 * tolerance / curvature);
}

double stationParameter(double minimum, double maximum, std::size_t station, std::size_t stationCount)
{
    return minimum + (maximum - minimum) * static_cast<double>(station) / static_cast<double>(stationCount);
}

double boundingBoxDiagonal(const std::vector<FeatureSample>& samples)
{
    if (samples.empty()) return 0.0;

    Point3D minimum = samples.front().position;
    Point3D maximum = samples.front().position;

    for (auto& sample : samples)
    {
        minimum = minimum.cwiseMin(sample.position);
        maximum = maximum.cwiseMax(sample.position);
    }

    return (maximum - minimum).norm();
}

void appendCurveSamples(const Geometry3D::GeometryCollection3D& geometry,
                        const Topology3D::Topology3D& topology,
                        const SizingFieldSettings3D& settings,
                        GeometrySamples& result)
{
    auto stations = std::max<std::size_t>(settings.samplesPerCurve, 2);

    for (auto& edgeId : topology.getAllEdgeIds())
    {
        // A seam twin would only repeat its original's samples, and a degenerate edge has no
        // real 3D curve to probe.
        if (topology.getSeamCollection().isSeamTwin(edgeId)) continue;

        auto edge = geometry.getEdge(edgeId);
        if (!edge || edge->isDegenerate()) continue;

        auto [tMinimum, tMaximum] = edge->getParameterBounds();
        double arcLength = 0.0;
        Point3D previous = edge->getPoint(tMinimum);

        for (std::size_t station = 0; station <= stations; ++station)
        {
            double t = stationParameter(tMinimum, tMaximum, station, stations);
            Point3D position = edge->getPoint(t);
            arcLength += (position - previous).norm();
            previous = position;

            result.samples.push_back({position, edgeId, arcLength, true});
            result.curvatureSizes.push_back(
                sizeFromCurvature(edge->getCurvature(t), settings.chordDeviationTolerance));
        }
    }
}

void appendSurfaceSamples(const Geometry3D::GeometryCollection3D& geometry,
                          const Topology3D::Topology3D& topology,
                          const SizingFieldSettings3D& settings,
                          GeometrySamples& result)
{
    auto stations = std::max<std::size_t>(settings.samplesPerSurfaceDirection, 2);

    for (auto& surfaceId : topology.getAllSurfaceIds())
    {
        auto surface = geometry.getSurface(surfaceId);
        if (!surface) continue;

        auto bounds = surface->getParameterBounds();

        for (std::size_t uStation = 0; uStation <= stations; ++uStation)
        {
            double u = stationParameter(bounds.getUMin(), bounds.getUMax(), uStation, stations);

            for (std::size_t vStation = 0; vStation <= stations; ++vStation)
            {
                double v = stationParameter(bounds.getVMin(), bounds.getVMax(), vStation, stations);

                // The untrimmed parameter grid can land in a hole or cutout (OPE-169); such a point
                // is not on the model and must not seed the field.
                if (!surface->isUVWithinTrimmedBoundary(u, v)) continue;

                auto curvatures = surface->getPrincipalCurvatures(u, v);
                double curvature = curvatures.has_value() ? std::max(std::abs(curvatures->minimum), std::abs(curvatures->maximum)) : 0.0;

                result.samples.push_back({surface->getPoint(u, v), surfaceId, 0.0, false});
                result.curvatureSizes.push_back(sizeFromCurvature(curvature, settings.chordDeviationTolerance));
            }
        }
    }
}

std::vector<SizingSource> buildSources(const GeometrySamples& geometrySamples,
                                       const std::vector<double>& featureSizes,
                                       const SizingFieldSettings3D& settings)
{
    auto& samples = geometrySamples.samples;
    double diagonal = boundingBoxDiagonal(samples);
    double sizeFloor = settings.minimumSizeFraction * diagonal;

    std::vector<SizingSource> sources;
    sources.reserve(samples.size());

    for (std::size_t i = 0; i < samples.size(); ++i)
    {
        double featureSize = featureSizes[i] / settings.elementsAcrossFeatureGap;
        double size = settings.includeCurvature ? std::min(geometrySamples.curvatureSizes[i], featureSize) : featureSize;

        // Flat geometry with nothing unrelated nearby: no source is the right answer, and the
        // envelope covers the position from its neighbours.
        if (!std::isfinite(size)) continue;

        sources.push_back({samples[i].position, std::max(size, sizeFloor)});
    }

    // The geometry imposes no size limit anywhere: one coarse source at model scale says so and
    // keeps every consumer on the same path. A source too large is safe; too small is not.
    if (sources.empty()) sources.push_back({Point3D::Zero(), diagonal > 0.0 ? diagonal : 1.0});

    return sources;
}

} // namespace

} // namespace Meshing
