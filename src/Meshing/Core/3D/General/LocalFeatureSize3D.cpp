#include "Meshing/Core/3D/General/LocalFeatureSize3D.h"

#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ICorner3D.h"
#include "Topology/Topology3D.h"

#include <algorithm>
#include <limits>
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

/// How far apart two samples on one curve are, measured along that curve.
/// On a CLOSED curve the route may run either way round, and the shorter
/// one wins -- without which a seam circle's first and last samples, which
/// occupy the same point in space, would appear a full circumference apart
/// along the curve and so read as an infinitely thin feature.
double arcDistance(double arcLengthA, double arcLengthB, double totalArcLength, bool closed)
{
    const double along = std::abs(arcLengthA - arcLengthB);
    return closed ? std::min(along, totalArcLength - along) : along;
}

/// The boundary edges of one entity. A surface reports the edges bounding
/// it; an edge reports itself, since it is its own boundary as far as
/// routing between entities is concerned.
std::vector<std::string> entityBoundaryEdgeIds(const std::string& entityId,
                                               const Topology3D::Topology3D& topology,
                                               const std::unordered_set<std::string>& edgeIds,
                                               const std::unordered_set<std::string>& surfaceIds)
{
    if (edgeIds.count(entityId) > 0)
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
                                         const std::unordered_set<std::string>& edgeIds,
                                         const std::unordered_set<std::string>& surfaceIds)
{
    if (edgeIds.count(entityId) > 0)
    {
        const auto& edge = topology.getEdge(entityId);
        return {edge.getStartCornerId(), edge.getEndCornerId()};
    }

    if (surfaceIds.count(entityId) > 0)
        return topology.getSurface(entityId).getCornerIds();

    return {};
}

} // namespace

std::vector<double> LocalFeatureSize3D::compute(const std::vector<FeatureSample>& samples,
                                                const Geometry3D::GeometryCollection3D& geometry,
                                                const Topology3D::Topology3D& topology)
{
    const auto allEdgeIds = topology.getAllEdgeIds();
    const auto allSurfaceIds = topology.getAllSurfaceIds();
    const std::unordered_set<std::string> edgeIds(allEdgeIds.begin(), allEdgeIds.end());
    const std::unordered_set<std::string> surfaceIds(allSurfaceIds.begin(), allSurfaceIds.end());

    // Every route between two different entities runs through an edge they
    // have in common, so the pairwise loop needs each sample's distance to
    // each edge. Resolving that here keeps the loop itself to arithmetic.
    std::unordered_map<std::string, std::size_t> edgeIndexById;
    for (std::size_t index = 0; index < allEdgeIds.size(); ++index)
        edgeIndexById.emplace(allEdgeIds[index], index);

    std::unordered_map<std::string, std::vector<std::size_t>> boundaryEdgeIndicesByEntity;
    std::unordered_map<std::string, std::vector<std::string>> cornerIdsByEntity;
    std::unordered_map<std::string, Point3D> cornerPositions;

    for (const auto& sample : samples)
    {
        if (boundaryEdgeIndicesByEntity.count(sample.entityId) > 0)
            continue;

        std::vector<std::size_t> indices;
        for (const auto& edgeId :
             entityBoundaryEdgeIds(sample.entityId, topology, edgeIds, surfaceIds))
        {
            const auto found = edgeIndexById.find(edgeId);
            if (found != edgeIndexById.end())
                indices.push_back(found->second);
        }

        boundaryEdgeIndicesByEntity.emplace(sample.entityId, std::move(indices));

        auto cornerIds = entityCornerIds(sample.entityId, topology, edgeIds, surfaceIds);
        for (const auto& cornerId : cornerIds)
        {
            if (cornerPositions.count(cornerId) > 0)
                continue;

            if (const auto* corner = geometry.getCorner(cornerId))
                cornerPositions.emplace(cornerId, corner->getPoint());
        }

        cornerIdsByEntity.emplace(sample.entityId, std::move(cornerIds));
    }

    // Each curve's own length, and whether it closes on itself -- both are
    // needed to measure distance along it (see arcDistance).
    struct CurveExtent
    {
        double totalArcLength = 0.0;
        Point3D firstPosition = Point3D::Zero();
        Point3D lastPosition = Point3D::Zero();
        bool started = false;
    };

    std::unordered_map<std::string, CurveExtent> curveExtents;

    for (const auto& sample : samples)
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

    std::unordered_map<std::string, bool> curveIsClosed;
    for (const auto& [entityId, extent] : curveExtents)
    {
        curveIsClosed[entityId] =
            (extent.firstPosition - extent.lastPosition).norm() <=
            CLOSED_CURVE_TOLERANCE * std::max(extent.totalArcLength, 1.0);
    }

    const double infinity = std::numeric_limits<double>::infinity();
    std::vector<std::vector<double>> distanceToEdge(samples.size(),
                                                    std::vector<double>(allEdgeIds.size(), infinity));

    for (std::size_t i = 0; i < samples.size(); ++i)
    {
        for (const auto& other : samples)
        {
            if (!other.onCurve)
                continue;

            const auto edge = edgeIndexById.find(other.entityId);
            if (edge == edgeIndexById.end())
                continue;

            distanceToEdge[i][edge->second] =
                std::min(distanceToEdge[i][edge->second], (samples[i].position - other.position).norm());
        }
    }

    std::vector<double> featureSizes(samples.size(), infinity);

    for (std::size_t i = 0; i < samples.size(); ++i)
    {
        for (std::size_t j = i + 1; j < samples.size(); ++j)
        {
            const auto& a = samples[i];
            const auto& b = samples[j];
            const double distance = (a.position - b.position).norm();

            if (distance <= 0.0)
                continue;

            const auto& edgesA = boundaryEdgeIndicesByEntity.at(a.entityId);
            const auto& edgesB = boundaryEdgeIndicesByEntity.at(b.entityId);

            // A curve that bounds a surface lies ON it. The two are close
            // everywhere along their whole shared length, structurally, so
            // no separation between them ever describes a thin feature.
            const bool oneBoundsTheOther =
                (a.entityId != b.entityId) &&
                ((edgesA.size() == 1 && std::find(edgesB.begin(), edgesB.end(), edgesA.front()) != edgesB.end()) ||
                 (edgesB.size() == 1 && std::find(edgesA.begin(), edgesA.end(), edgesB.front()) != edgesA.end()));

            if (oneBoundsTheOther)
                continue;

            // Length of the shortest route between the samples that stays on
            // the geometry, as far as it can be established here. Infinity
            // means no connecting route is known, so the straight line is
            // unconditionally a shortcut.
            double pathLength = infinity;

            if (a.entityId == b.entityId)
            {
                // Same curve: the route is the curve itself. Same surface:
                // no geodesic estimate is available, so treat the samples as
                // connected and never constraining (header's limitation).
                pathLength = (a.onCurve && b.onCurve) ? arcDistance(a.arcLengthAlongCurve,
                                                                    b.arcLengthAlongCurve,
                                                                    curveExtents.at(a.entityId).totalArcLength,
                                                                    curveIsClosed.at(a.entityId)) :
                                                        0.0;
            }
            else
            {
                // Different entities: the route runs through whatever they
                // have in common -- a shared bounding edge, or just a shared
                // corner. Take the cheapest such junction.
                for (const std::size_t edge : edgesA)
                {
                    if (std::find(edgesB.begin(), edgesB.end(), edge) == edgesB.end())
                        continue;

                    pathLength =
                        std::min(pathLength, distanceToEdge[i][edge] + distanceToEdge[j][edge]);
                }

                const auto& cornersB = cornerIdsByEntity.at(b.entityId);
                for (const auto& cornerId : cornerIdsByEntity.at(a.entityId))
                {
                    if (std::find(cornersB.begin(), cornersB.end(), cornerId) == cornersB.end())
                        continue;

                    const auto position = cornerPositions.find(cornerId);
                    if (position == cornerPositions.end())
                        continue;

                    pathLength = std::min(pathLength,
                                          (a.position - position->second).norm() +
                                              (b.position - position->second).norm());
                }
            }

            if (pathLength <= PROXIMITY_PATH_FACTOR * distance)
                continue;

            featureSizes[i] = std::min(featureSizes[i], distance);
            featureSizes[j] = std::min(featureSizes[j], distance);
        }
    }

    return featureSizes;
}

} // namespace Meshing
