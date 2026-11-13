#include "Meshing/Core/3D/General/LocalFeatureSize3D.h"

#include "Topology/Topology3D.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <unordered_map>
#include <unordered_set>

namespace Meshing
{

namespace
{

// How much longer the route along the geometry must be than the straight
// line between two samples before they count as unrelated -- i.e. before
// the straight line counts as a genuine shortcut through space.
//
// At 1.0 every sample would be unrelated to its own neighbours (along a
// straight curve, route length equals chord length). The margin must sit
// comfortably above 1 so that ordinary curvature and ordinary junctions do
// not read as thin features: two curves meeting at a right angle, sampled
// at equal distance r from the corner, have a route of 2r against a
// separation of sqrt(2)*r, a ratio of only 1.41. Genuine thin features are
// far above that -- a plate of thickness t sampled at distance L in from
// its rim gives 2L/t, which grows without bound as you move inward.
constexpr double PROXIMITY_PATH_FACTOR = 3.0;

// How close a curve's two ends must be, relative to its own length, for it
// to count as closed. A seam circle closes exactly; the margin only absorbs
// round-trip round-off in the sampled positions.
constexpr double CLOSED_CURVE_TOLERANCE = 1e-9;

/// The edges and corners through which one entity connects to the others.
struct EntityConnections
{
    std::vector<std::size_t> boundaryEdgeIndices;
    std::vector<std::string> cornerIds;
};

/// One curve's own length, and whether it closes on itself -- both are
/// needed to measure distance along it (see arcDistance).
struct CurveExtent
{
    double totalArcLength = 0.0;
    Point3D firstPosition = Point3D::Zero();
    Point3D lastPosition = Point3D::Zero();
    bool started = false;
    bool closed = false;
};

std::unordered_map<std::string, EntityConnections>
collectEntityConnections(const std::vector<FeatureSample>& samples,
                         const Topology3D::Topology3D& topology,
                         const std::unordered_map<std::string, std::size_t>& edgeIndexById);
std::unordered_map<std::string, CurveExtent> measureCurveExtents(const std::vector<FeatureSample>& samples);
std::vector<std::vector<double>> computeDistancesToEdges(const std::vector<FeatureSample>& samples,
                                                         const std::unordered_map<std::string, std::size_t>& edgeIndexById,
                                                         std::size_t edgeCount);
double routeAlongEntity(const FeatureSample& sample0,
                        const FeatureSample& sample1,
                        const std::unordered_map<std::string, CurveExtent>& curveExtents);
bool oneBoundsTheOther(const std::vector<std::size_t>& edges0, const std::vector<std::size_t>& edges1);
std::optional<double> routeThroughSharedEdges(const std::vector<std::size_t>& edges0,
                                              const std::vector<std::size_t>& edges1,
                                              const std::vector<double>& distancesToEdges0,
                                              const std::vector<double>& distancesToEdges1);
bool shareACorner(const std::vector<std::string>& corners0, const std::vector<std::string>& corners1);

} // namespace

std::vector<double> LocalFeatureSize3D::compute(const std::vector<FeatureSample>& samples,
                                                const Topology3D::Topology3D& topology)
{
    // Every route between two different entities runs through an edge they
    // have in common, so the pairwise loop needs each sample's distance to
    // each edge. Resolving that here keeps the loop itself to arithmetic.
    auto allEdgeIds = topology.getAllEdgeIds();
    std::unordered_map<std::string, std::size_t> edgeIndexById;
    for (std::size_t index = 0; index < allEdgeIds.size(); ++index)
        edgeIndexById.emplace(allEdgeIds[index], index);

    auto connectionsByEntity = collectEntityConnections(samples, topology, edgeIndexById);
    auto curveExtents = measureCurveExtents(samples);
    auto distanceToEdge = computeDistancesToEdges(samples, edgeIndexById, allEdgeIds.size());

    auto infinity = std::numeric_limits<double>::infinity();
    std::vector<double> featureSizes(samples.size(), infinity);

    for (std::size_t index0 = 0; index0 < samples.size(); ++index0)
    {
        for (std::size_t index1 = index0 + 1; index1 < samples.size(); ++index1)
        {
            auto& sample0 = samples[index0];
            auto& sample1 = samples[index1];
            auto distance = (sample0.position - sample1.position).norm();

            if (distance <= 0.0)
                continue;

            // Length of the shortest route between the samples that stays on
            // the geometry, as far as it can be established here. Infinity
            // means no connecting route is known, so the straight line is
            // unconditionally a shortcut.
            double pathLength = infinity;

            if (sample0.entityId == sample1.entityId)
            {
                pathLength = routeAlongEntity(sample0, sample1, curveExtents);
            }
            else
            {
                auto& connections0 = connectionsByEntity.at(sample0.entityId);
                auto& connections1 = connectionsByEntity.at(sample1.entityId);

                if (oneBoundsTheOther(connections0.boundaryEdgeIndices, connections1.boundaryEdgeIndices))
                    continue;

                auto route = routeThroughSharedEdges(connections0.boundaryEdgeIndices,
                                                     connections1.boundaryEdgeIndices,
                                                     distanceToEdge[index0],
                                                     distanceToEdge[index1]);

                // Entities meeting at only a CORNER are deliberately never
                // constrained against each other, however close they run.
                //
                // Two curves leaving a shared corner at angle theta, sampled
                // at distance r along each, are 2*r*sin(theta/2) apart with a
                // route of 2*r -- so the route test fires for every pair once
                // theta drops below about 39 degrees, all the way down, while
                // the separation itself tends to zero at the corner. Local
                // feature size is therefore unbounded below in any wedge, and
                // what it actually reports is the sampling density rather
                // than the geometry: measured on SaddleSurfaceMesh, whose
                // corner wedges are about 15 degrees, the field's finest size
                // halved exactly as samplesPerCurve doubled (0.1136, 0.0573,
                // 0.0289, 0.0145 for 16, 32, 64, 128).
                //
                // Sizing cannot fix that, because a closing wedge cannot be
                // resolved by refinement at all -- it is the classic small
                // input angle case, which this codebase already handles where
                // it belongs, via the acute-angle splitting cascade
                // terminating at minimumEdgeLength (see
                // SurfaceMesh3DQualitySettings::maxRefinementIterations).
                // Letting it into h(x) instead made SaddleSurfaceMesh diverge:
                // 8991 nodes after 90 minutes against a 1223-node baseline,
                // still growing.
                if (!route && shareACorner(connections0.cornerIds, connections1.cornerIds))
                    continue;

                pathLength = route.value_or(infinity);
            }

            if (pathLength <= PROXIMITY_PATH_FACTOR * distance)
                continue;

            featureSizes[index0] = std::min(featureSizes[index0], distance);
            featureSizes[index1] = std::min(featureSizes[index1], distance);
        }
    }

    return featureSizes;
}

namespace
{

template <typename T>
bool contains(const std::vector<T>& values, const T& value)
{
    return std::find(values.begin(), values.end(), value) != values.end();
}

/// How far apart two samples on one curve are, measured along that curve.
/// On a CLOSED curve the route may run either way round, and the shorter
/// one wins -- without which a seam circle's first and last samples, which
/// occupy the same point in space, would appear a full circumference apart
/// along the curve and so read as an infinitely thin feature.
double arcDistance(double arcLength0, double arcLength1, double totalArcLength, bool closed)
{
    auto along = std::abs(arcLength0 - arcLength1);
    return closed ? std::min(along, totalArcLength - along) : along;
}

/// The boundary edges of one entity. A surface reports the edges bounding
/// it; an edge reports itself, since it is its own boundary as far as
/// routing between entities is concerned.
std::vector<std::string> entityBoundaryEdgeIds(const std::string& entityId,
                                               const Topology3D::Topology3D& topology,
                                               const std::unordered_map<std::string, std::size_t>& edgeIndexById,
                                               const std::unordered_set<std::string>& surfaceIds)
{
    if (edgeIndexById.count(entityId) > 0)
        return {entityId};

    if (surfaceIds.count(entityId) > 0)
        return topology.getSurface(entityId).getBoundaryEdgeIds();

    return {};
}

/// The corners of one entity. Needed alongside the edges above because two
/// entities can meet at a single corner without sharing any edge -- two
/// edges of a box at a vertex, or an edge touching a face it does not
/// bound. Such a pair does have a route, and treating it as unconnected
/// makes every junction look like a thin feature.
std::vector<std::string> entityCornerIds(const std::string& entityId,
                                         const Topology3D::Topology3D& topology,
                                         const std::unordered_map<std::string, std::size_t>& edgeIndexById,
                                         const std::unordered_set<std::string>& surfaceIds)
{
    if (edgeIndexById.count(entityId) > 0)
    {
        auto& edge = topology.getEdge(entityId);
        return {edge.getStartCornerId(), edge.getEndCornerId()};
    }

    if (surfaceIds.count(entityId) > 0)
        return topology.getSurface(entityId).getCornerIds();

    return {};
}

std::unordered_map<std::string, EntityConnections>
collectEntityConnections(const std::vector<FeatureSample>& samples,
                         const Topology3D::Topology3D& topology,
                         const std::unordered_map<std::string, std::size_t>& edgeIndexById)
{
    auto allSurfaceIds = topology.getAllSurfaceIds();
    std::unordered_set<std::string> surfaceIds(allSurfaceIds.begin(), allSurfaceIds.end());

    std::unordered_map<std::string, EntityConnections> connectionsByEntity;
    for (auto& sample : samples)
    {
        if (connectionsByEntity.count(sample.entityId) > 0)
            continue;

        EntityConnections connections;
        for (auto& edgeId : entityBoundaryEdgeIds(sample.entityId, topology, edgeIndexById, surfaceIds))
        {
            auto found = edgeIndexById.find(edgeId);
            if (found != edgeIndexById.end())
                connections.boundaryEdgeIndices.push_back(found->second);
        }
        connections.cornerIds = entityCornerIds(sample.entityId, topology, edgeIndexById, surfaceIds);

        connectionsByEntity.emplace(sample.entityId, std::move(connections));
    }

    return connectionsByEntity;
}

std::unordered_map<std::string, CurveExtent> measureCurveExtents(const std::vector<FeatureSample>& samples)
{
    std::unordered_map<std::string, CurveExtent> curveExtents;

    for (auto& sample : samples)
    {
        if (!sample.onCurve)
            continue;

        auto& extent = curveExtents[sample.entityId];
        if (!extent.started)
        {
            extent.firstPosition = sample.position;
            extent.started = true;
        }

        extent.lastPosition = sample.position;
        extent.totalArcLength = std::max(extent.totalArcLength, sample.arcLengthAlongCurve);
    }

    for (auto& [entityId, extent] : curveExtents)
    {
        extent.closed = (extent.firstPosition - extent.lastPosition).norm() <=
                        CLOSED_CURVE_TOLERANCE * std::max(extent.totalArcLength, 1.0);
    }

    return curveExtents;
}

/// For each sample, the distance to the nearest sample on each edge, or
/// infinity for an edge with no samples.
std::vector<std::vector<double>> computeDistancesToEdges(const std::vector<FeatureSample>& samples,
                                                         const std::unordered_map<std::string, std::size_t>& edgeIndexById,
                                                         std::size_t edgeCount)
{
    std::vector<std::vector<double>> distanceToEdge(
        samples.size(), std::vector<double>(edgeCount, std::numeric_limits<double>::infinity()));

    for (std::size_t sampleIndex = 0; sampleIndex < samples.size(); ++sampleIndex)
    {
        for (auto& other : samples)
        {
            if (!other.onCurve)
                continue;

            auto edge = edgeIndexById.find(other.entityId);
            if (edge == edgeIndexById.end())
                continue;

            auto& distance = distanceToEdge[sampleIndex][edge->second];
            distance = std::min(distance, (samples[sampleIndex].position - other.position).norm());
        }
    }

    return distanceToEdge;
}

/// The route between two samples on the same entity. Same curve: the route
/// is the curve itself. Same surface: no geodesic estimate is available, so
/// the samples are treated as connected and never constraining (see the
/// header's known limitation).
double routeAlongEntity(const FeatureSample& sample0,
                        const FeatureSample& sample1,
                        const std::unordered_map<std::string, CurveExtent>& curveExtents)
{
    if (!sample0.onCurve || !sample1.onCurve)
        return 0.0;

    auto& extent = curveExtents.at(sample0.entityId);
    return arcDistance(sample0.arcLengthAlongCurve, sample1.arcLengthAlongCurve, extent.totalArcLength, extent.closed);
}

/// A curve that bounds a surface lies ON it. The two are close everywhere
/// along their whole shared length, structurally, so no separation between
/// them ever describes a thin feature.
bool oneBoundsTheOther(const std::vector<std::size_t>& edges0, const std::vector<std::size_t>& edges1)
{
    return (edges0.size() == 1 && contains(edges1, edges0.front())) ||
           (edges1.size() == 1 && contains(edges0, edges1.front()));
}

/// The shortest route between samples on two entities meeting along a
/// shared EDGE, which runs through that edge -- this is what lets a thin fin
/// register: its two faces meet at the tip edge, yet mid-fin the route out
/// to that edge and back is far longer than the thickness. No value when the
/// entities share no edge.
std::optional<double> routeThroughSharedEdges(const std::vector<std::size_t>& edges0,
                                              const std::vector<std::size_t>& edges1,
                                              const std::vector<double>& distancesToEdges0,
                                              const std::vector<double>& distancesToEdges1)
{
    std::optional<double> shortest;
    for (auto edge : edges0)
    {
        if (!contains(edges1, edge)) continue;

        auto route = distancesToEdges0[edge] + distancesToEdges1[edge];
        shortest = std::min(shortest.value_or(std::numeric_limits<double>::infinity()), route);
    }
    return shortest;
}

bool shareACorner(const std::vector<std::string>& corners0, const std::vector<std::string>& corners1)
{
    return std::any_of(corners0.begin(),
                       corners0.end(),
                       [&](const std::string& cornerId)
                       { return contains(corners1, cornerId); });
}

} // namespace

} // namespace Meshing
