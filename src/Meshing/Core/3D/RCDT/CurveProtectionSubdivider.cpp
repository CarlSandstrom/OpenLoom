#include "Meshing/Core/3D/RCDT/CurveProtectionSubdivider.h"

#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/IEdge3D.h"
#include "Meshing/Core/3D/General/DiscretizationResult3D.h"
#include "Meshing/Core/3D/RCDT/CurveProtectionScheme.h"
#include "Topology/SeamCollection.h"
#include "Topology/Topology3D.h"
#include "spdlog/spdlog.h"

#include <algorithm>
#include <unordered_set>

namespace Meshing
{

namespace
{

// A genuine bug (rather than the size floor) looping forever would hang the
// mesher outright; this is a defensive cap far above any realistic
// violation count, not a normal exit path -- the size floor in trySplit()
// is what's expected to terminate every real run.
constexpr int MAX_SUBDIVISION_ITERATIONS = 10000;

std::map<std::string, std::vector<size_t>> buildFilteredEdgeMap(
    const DiscretizationResult3D& discretizationResult,
    const Topology3D::Topology3D& topology,
    const Geometry3D::GeometryCollection3D& geometry)
{
    std::map<std::string, std::vector<size_t>> filtered;
    for (const auto& [edgeId, chain] : discretizationResult.edgeIdToPointIndicesMap)
    {
        if (topology.getSeamCollection().isSeamTwin(edgeId))
            continue;
        const Geometry3D::IEdge3D* edge = geometry.getEdge(edgeId);
        if (edge && edge->isDegenerate())
            continue;
        filtered[edgeId] = chain;
    }
    return filtered;
}

// Splits the segment (violation.nodeId1, violation.nodeId2) at its
// arc-length midpoint on the true curve, appending the new point to
// discretizationResult and splicing it into the edge's chain. Returns false
// (does nothing) if the edge can't be resolved or either resulting
// sub-segment would fall below minimumEdgeLength -- the caller treats that
// as "leave this violation permanently unresolved."
bool trySplit(const UnresolvedProtectionSegment& violation,
             DiscretizationResult3D& discretizationResult,
             const Geometry3D::GeometryCollection3D& geometry,
             double minimumEdgeLength)
{
    const Geometry3D::IEdge3D* edge = geometry.getEdge(violation.edgeId);
    if (!edge)
        return false;

    auto& chain = discretizationResult.edgeIdToPointIndicesMap.at(violation.edgeId);
    const auto positionIt = std::find(chain.begin(), chain.end(), violation.nodeId1);
    if (positionIt == chain.end() || positionIt + 1 == chain.end() || *(positionIt + 1) != violation.nodeId2)
        return false;
    const size_t k = static_cast<size_t>(positionIt - chain.begin());

    // Same t-value convention buildCurveSegments() uses: a chain's first and
    // last positions are the edge's own parameter bounds (its corners), any
    // interior position reads its own stored edge parameter.
    const auto [tMin, tMax] = edge->getParameterBounds();
    const double t1 = (k == 0) ? tMin : discretizationResult.edgeParameters[chain[k]][0];
    const double t2 = (k + 1 == chain.size() - 1) ? tMax : discretizationResult.edgeParameters[chain[k + 1]][0];
    const double tMid = edge->getParameterAtArcLengthFraction(t1, t2, 0.5);
    const Point3D newPoint = edge->getPoint(tMid);

    const double distanceToFirst = (newPoint - discretizationResult.points[chain[k]]).norm();
    const double distanceToSecond = (newPoint - discretizationResult.points[chain[k + 1]]).norm();
    if (distanceToFirst < minimumEdgeLength || distanceToSecond < minimumEdgeLength)
        return false;

    const size_t newIndex = discretizationResult.points.size();
    discretizationResult.points.push_back(newPoint);
    discretizationResult.edgeParameters.push_back({tMid});
    discretizationResult.geometryIds.push_back({violation.edgeId});
    chain.insert(chain.begin() + static_cast<std::ptrdiff_t>(k) + 1, newIndex);
    return true;
}

} // namespace

std::unordered_map<size_t, double> CurveProtectionSubdivider::subdivide(
    DiscretizationResult3D& discretizationResult,
    const Topology3D::Topology3D& topology,
    const Geometry3D::GeometryCollection3D& geometry,
    double minimumEdgeLength)
{
    std::unordered_set<size_t> cornerPointIndices;
    for (const auto& [cornerId, pointIndex] : discretizationResult.cornerIdToPointIndexMap)
        cornerPointIndices.insert(pointIndex);

    // Points permanently unresolved (hit the size floor): tracked so a
    // repeat violation on the same pair doesn't get retried every
    // iteration -- same "mark unrefinable, don't retry forever" pattern
    // RCDTRefiner uses for its own size-floor cases.
    std::unordered_set<size_t> unresolvable; // keyed by nodeId1 ^ (nodeId2 << 1), see below
    auto pairKey = [](size_t a, size_t b) { return a < b ? (a << 20) ^ b : (b << 20) ^ a; };

    std::unordered_map<size_t, double> weights;
    for (int iteration = 0; iteration < MAX_SUBDIVISION_ITERATIONS; ++iteration)
    {
        const auto filteredEdges = buildFilteredEdgeMap(discretizationResult, topology, geometry);
        weights = CurveProtectionScheme::computeWeights(filteredEdges, cornerPointIndices, discretizationResult.points);
        const auto violations =
            CurveProtectionScheme::findUnresolvedSegments(filteredEdges, weights, discretizationResult.points);

        bool progressed = false;
        for (const auto& violation : violations)
        {
            if (unresolvable.contains(pairKey(violation.nodeId1, violation.nodeId2)))
                continue;

            if (trySplit(violation, discretizationResult, geometry, minimumEdgeLength))
            {
                progressed = true;
                break; // chain indices shifted; restart from a fresh computeWeights() next iteration
            }
            unresolvable.insert(pairKey(violation.nodeId1, violation.nodeId2));
        }

        if (!progressed)
        {
            for (const auto& violation : violations)
            {
                spdlog::warn("CurveProtectionSubdivider::subdivide: edge '{}' between points {} and {} could not "
                            "be resolved -- the next split would fall below the minimum edge length",
                            violation.edgeId, violation.nodeId1, violation.nodeId2);
            }
            return weights;
        }
    }

    spdlog::warn("CurveProtectionSubdivider::subdivide: reached the iteration cap ({}) without resolving every "
                "violation -- this should not happen given the size floor, investigate as a bug",
                MAX_SUBDIVISION_ITERATIONS);
    return weights;
}

} // namespace Meshing
