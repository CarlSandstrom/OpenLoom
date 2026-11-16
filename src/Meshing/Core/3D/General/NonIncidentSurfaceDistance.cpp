#include "Meshing/Core/3D/General/NonIncidentSurfaceDistance.h"

#include "Common/BoundingBox2D.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/IEdge3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Topology/SeamCollection.h"
#include "Topology/Topology3D.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace Meshing
{

namespace
{

// Coarse samples per edge and per surface parameter direction. The surface
// samples sit at cell centres, the edge samples at the interval ends.
constexpr std::size_t COARSE_INTERVALS = 8;

// Refinement stops once the cell is this fraction of the distance: the result
// sizes a protecting ball, which does not need the distance to more digits.
constexpr double CONVERGENCE_FRACTION = 0.01;
constexpr int MAXIMUM_HALVINGS = 12;

using SampledEdge = NonIncidentSurfaceDistance::SampledEdge;
using SampledSurface = NonIncidentSurfaceDistance::SampledSurface;
using EdgeSample = NonIncidentSurfaceDistance::EdgeSample;
using SurfaceSample = NonIncidentSurfaceDistance::SurfaceSample;

SampledEdge sampleEdge(const Geometry3D::IEdge3D& edge);
std::vector<SurfaceSample> sampleSurface(const Geometry3D::ISurface3D& surface, Point2D& parameterStep);
bool sharesCorner(const SampledSurface& surface, const std::vector<std::string>& cornerIds);
double refineOnEdge(const SampledEdge& edge, const EdgeSample& sample, const Point3D& point, double best);
double refineOnSurface(const SampledSurface& surface, const SurfaceSample& sample, const Point3D& point, double best);

} // namespace

NonIncidentSurfaceDistance::NonIncidentSurfaceDistance(const Geometry3D::GeometryCollection3D& geometry,
                                                       const Topology3D::Topology3D& topology)
{
    std::unordered_map<std::string, std::size_t> edgeIndexOf;
    for (auto& edgeId : topology.getAllEdgeIds())
    {
        if (topology.getSeamCollection().isSeamTwin(edgeId)) continue;
        auto edge = geometry.getEdge(edgeId);
        if (edge->isDegenerate()) continue;
        edgeIndexOf[edgeId] = edges_.size();
        edges_.push_back(sampleEdge(*edge));
    }

    for (auto& surfaceId : topology.getAllSurfaceIds())
    {
        auto& topologySurface = topology.getSurface(surfaceId);
        SampledSurface sampled{geometry.getSurface(surfaceId), topologySurface.getCornerIds(), {}, {}, {}};
        for (auto& edgeId : topologySurface.getBoundaryEdgeIds())
        {
            auto found = edgeIndexOf.find(edgeId);
            if (found != edgeIndexOf.end()) sampled.edgeIndices.push_back(found->second);
        }
        sampled.samples = sampleSurface(*sampled.surface, sampled.parameterStep);
        surfaces_.push_back(std::move(sampled));
    }
}

double NonIncidentSurfaceDistance::distance(const Point3D& point, const std::vector<std::string>& cornerIds) const
{
    std::vector<const SampledSurface*> constraining;
    std::vector<bool> edgeConstrains(edges_.size(), false);
    for (auto& surface : surfaces_)
    {
        if (sharesCorner(surface, cornerIds)) continue;
        constraining.push_back(&surface);
        for (auto edgeIndex : surface.edgeIndices)
            edgeConstrains[edgeIndex] = true;
    }

    // Every coarse sample is itself a point of a constraining surface, so the
    // nearest of them bounds the answer before anything is refined.
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t edgeIndex = 0; edgeIndex < edges_.size(); ++edgeIndex)
    {
        if (!edgeConstrains[edgeIndex]) continue;
        for (auto& sample : edges_[edgeIndex].samples)
            best = std::min(best, (sample.point - point).norm());
    }
    for (auto surface : constraining)
        for (auto& sample : surface->samples)
            best = std::min(best, (sample.point - point).norm());

    // A sample is worth refining only while its cell could still hold a point
    // nearer than the best found. Edges go first: they settle the trimmed
    // boundary, and lower the bound the surface interiors have to beat.
    for (std::size_t edgeIndex = 0; edgeIndex < edges_.size(); ++edgeIndex)
    {
        if (!edgeConstrains[edgeIndex]) continue;
        for (auto& sample : edges_[edgeIndex].samples)
            if ((sample.point - point).norm() - sample.cellRadius < best)
                best = std::min(best, refineOnEdge(edges_[edgeIndex], sample, point, best));
    }
    for (auto surface : constraining)
        for (auto& sample : surface->samples)
            if ((sample.point - point).norm() - sample.cellRadius < best)
                best = std::min(best, refineOnSurface(*surface, sample, point, best));
    return best;
}

namespace
{

/// The edge at the ends of COARSE_INTERVALS equal parameter intervals. A
/// sample's cell reaches half way to each neighbour.
SampledEdge sampleEdge(const Geometry3D::IEdge3D& edge)
{
    auto [start, end] = edge.getParameterBounds();
    SampledEdge sampled{&edge, (end - start) / static_cast<double>(COARSE_INTERVALS), {}};
    for (std::size_t i = 0; i <= COARSE_INTERVALS; ++i)
    {
        auto parameter = start + static_cast<double>(i) * sampled.parameterStep;
        sampled.samples.push_back({parameter, edge.getPoint(parameter), 0.0});
    }
    for (std::size_t i = 0; i <= COARSE_INTERVALS; ++i)
    {
        auto& sample = sampled.samples[i];
        if (i > 0) sample.cellRadius = (sample.point - sampled.samples[i - 1].point).norm() / 2.0;
        if (i < COARSE_INTERVALS)
            sample.cellRadius = std::max(sample.cellRadius, (sample.point - sampled.samples[i + 1].point).norm() / 2.0);
    }
    return sampled;
}

/// The surface at the centres of a COARSE_INTERVALS-square parameter grid,
/// keeping the samples inside the trimmed boundary. A sample's cell reaches
/// half way to each of its eight neighbours.
std::vector<SurfaceSample> sampleSurface(const Geometry3D::ISurface3D& surface, Point2D& parameterStep)
{
    auto bounds = surface.getParameterBounds();
    parameterStep = Point2D((bounds.getUMax() - bounds.getUMin()) / static_cast<double>(COARSE_INTERVALS),
                            (bounds.getVMax() - bounds.getVMin()) / static_cast<double>(COARSE_INTERVALS));
    if (!std::isfinite(parameterStep.x()) || !std::isfinite(parameterStep.y())) return {};

    auto count = static_cast<int>(COARSE_INTERVALS);
    auto parametersAt = [&](int column, int row)
    {
        return Point2D(bounds.getUMin() + (column + 0.5) * parameterStep.x(),
                       bounds.getVMin() + (row + 0.5) * parameterStep.y());
    };
    std::vector<Point3D> grid;
    for (int row = 0; row < count; ++row)
        for (int column = 0; column < count; ++column)
        {
            auto parameters = parametersAt(column, row);
            grid.push_back(surface.getPoint(parameters.x(), parameters.y()));
        }

    std::vector<SurfaceSample> samples;
    for (int row = 0; row < count; ++row)
        for (int column = 0; column < count; ++column)
        {
            auto parameters = parametersAt(column, row);
            if (!surface.isUVWithinTrimmedBoundary(parameters.x(), parameters.y())) continue;

            auto& centre = grid[row * count + column];
            double cellRadius = 0.0;
            for (int neighbourRow = std::max(row - 1, 0); neighbourRow <= std::min(row + 1, count - 1); ++neighbourRow)
                for (int neighbourColumn = std::max(column - 1, 0); neighbourColumn <= std::min(column + 1, count - 1);
                     ++neighbourColumn)
                    cellRadius = std::max(cellRadius, (grid[neighbourRow * count + neighbourColumn] - centre).norm() / 2.0);
            samples.push_back({parameters, centre, cellRadius});
        }
    return samples;
}

bool sharesCorner(const SampledSurface& surface, const std::vector<std::string>& cornerIds)
{
    return std::ranges::any_of(cornerIds, [&](const std::string& cornerId)
                               { return std::ranges::find(surface.cornerIds, cornerId) != surface.cornerIds.end(); });
}

/// The distance from @p point to the edge near @p sample: steps to the nearer
/// of the two points half a cell either side, halves the cell, and repeats.
/// Gives up once the cell can no longer hold a point nearer than @p best.
double refineOnEdge(const SampledEdge& edge, const EdgeSample& sample, const Point3D& point, double best)
{
    auto [lower, upper] = std::minmax(edge.edge->getParameterBounds().first, edge.edge->getParameterBounds().second);
    auto parameter = sample.parameter;
    auto nearest = (sample.point - point).norm();
    auto step = edge.parameterStep / 2.0;
    auto cellRadius = sample.cellRadius;
    for (int halving = 0; halving < MAXIMUM_HALVINGS; ++halving)
    {
        if (nearest - cellRadius >= best || cellRadius < CONVERGENCE_FRACTION * std::min(nearest, best)) break;

        auto centre = parameter;
        for (auto offset : {-step, step})
        {
            auto candidate = std::clamp(centre + offset, lower, upper);
            auto candidateDistance = (edge.edge->getPoint(candidate) - point).norm();
            if (candidateDistance >= nearest) continue;
            nearest = candidateDistance;
            parameter = candidate;
        }
        step /= 2.0;
        cellRadius /= 2.0;
    }
    return nearest;
}

/// As refineOnEdge, over the eight points half a cell around the current one
/// in the surface's parameter plane. Only points inside the trimmed boundary
/// are stepped to, so the result is always the distance to a point of the
/// trimmed surface.
double refineOnSurface(const SampledSurface& surface, const SurfaceSample& sample, const Point3D& point, double best)
{
    static constexpr std::array<std::array<double, 2>, 8> OFFSETS{
        {{-1.0, -1.0}, {0.0, -1.0}, {1.0, -1.0}, {-1.0, 0.0}, {1.0, 0.0}, {-1.0, 1.0}, {0.0, 1.0}, {1.0, 1.0}}};

    auto bounds = surface.surface->getParameterBounds();
    auto parameters = sample.parameters;
    auto nearest = (sample.point - point).norm();
    Point2D step = surface.parameterStep / 2.0;
    auto cellRadius = sample.cellRadius;
    for (int halving = 0; halving < MAXIMUM_HALVINGS; ++halving)
    {
        if (nearest - cellRadius >= best || cellRadius < CONVERGENCE_FRACTION * std::min(nearest, best)) break;

        auto centre = parameters;
        for (auto& offset : OFFSETS)
        {
            auto u = std::clamp(centre.x() + offset[0] * step.x(), bounds.getUMin(), bounds.getUMax());
            auto v = std::clamp(centre.y() + offset[1] * step.y(), bounds.getVMin(), bounds.getVMax());
            auto candidateDistance = (surface.surface->getPoint(u, v) - point).norm();
            if (candidateDistance >= nearest || !surface.surface->isUVWithinTrimmedBoundary(u, v)) continue;
            nearest = candidateDistance;
            parameters = Point2D(u, v);
        }
        step /= 2.0;
        cellRadius /= 2.0;
    }
    return nearest;
}

} // namespace

} // namespace Meshing
