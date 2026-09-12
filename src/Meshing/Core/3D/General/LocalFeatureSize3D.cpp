#include "Meshing/Core/3D/General/LocalFeatureSize3D.h"

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

        cornerIdsByEntity.emplace(sample.entityId,
                                  entityCornerIds(sample.entityId, topology, edgeIds, surfaceIds));
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

    auto shareACorner = [&](const std::string& entityA, const std::string& entityB)
    {
        const auto& cornersB = cornerIdsByEntity.at(entityB);
        for (const auto& cornerId : cornerIdsByEntity.at(entityA))
        {
            if (std::find(cornersB.begin(), cornersB.end(), cornerId) != cornersB.end())
                return true;
        }
        return false;
    };

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
                // Different entities meeting along a shared EDGE: the route
                // runs through it, which is what lets a thin fin register --
                // its two faces meet at the tip edge, yet mid-fin the route
                // out to that edge and back is far longer than the thickness.
                bool sharesEdge = false;
                for (const std::size_t edge : edgesA)
                {
                    if (std::find(edgesB.begin(), edgesB.end(), edge) == edgesB.end())
                        continue;

                    sharesEdge = true;
                    pathLength =
                        std::min(pathLength, distanceToEdge[i][edge] + distanceToEdge[j][edge]);
                }

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
                if (!sharesEdge && shareACorner(a.entityId, b.entityId))
                    continue;
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
