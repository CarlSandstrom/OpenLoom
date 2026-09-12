#include "Meshing/Core/3D/RCDT/CurveProtectionSubdivider.h"

#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/IEdge3D.h"
#include "Meshing/Core/3D/General/DiscretizationResult3D.h"
#include "Meshing/Core/3D/RCDT/CurveProtectionScheme.h"
#include "Topology/SeamCollection.h"
#include "Topology/Topology3D.h"
#include "spdlog/spdlog.h"

#include <algorithm>
#include <string>
#include <unordered_set>
#include <vector>

namespace Meshing
{

namespace
{

// A genuine bug (rather than the size floor) looping forever would hang the
// mesher outright; this is a defensive cap far above any realistic
// violation count, not a normal exit path -- the size floor in trySplit()
// is what's expected to terminate every real run.
constexpr int MAX_SUBDIVISION_ITERATIONS = 10000;

// The segments subdivide() has given up on: trySplit() declined them
// because the next split would fall below the size floor, so they are
// remembered and skipped instead of being retried on every iteration --
// the same "mark unrefinable, don't retry forever" pattern RCDTRefiner
// uses for its own size-floor cases. A segment is identified by its
// unordered pair of point indices, folded into a single key.
class AbandonedSegments
{
public:
    void add(const UnresolvedProtectionSegment& segment)
    {
        keys_.insert(keyFor(segment));
    }

    bool contains(const UnresolvedProtectionSegment& segment) const
    {
        return keys_.contains(keyFor(segment));
    }

private:
    static size_t keyFor(const UnresolvedProtectionSegment& segment)
    {
        const size_t smaller = std::min(segment.nodeId1, segment.nodeId2);
        const size_t larger = std::max(segment.nodeId1, segment.nodeId2);
        return (smaller << 20) ^ larger;
    }

    std::unordered_set<size_t> keys_;
};

// The discretized curve network CurveProtectionScheme is to work on: every
// discretized edge except the two categories that carry no curve of their
// own to protect -- a seam twin, which only duplicates its original edge's
// points in reverse, and a degenerate edge (e.g. a sphere's polar edge),
// which has no real length to subdivide.
std::map<std::string, std::vector<size_t>> collectCurveNetwork(
    const DiscretizationResult3D& discretizationResult,
    const Topology3D::Topology3D& topology,
    const Geometry3D::GeometryCollection3D& geometry)
{
    std::map<std::string, std::vector<size_t>> curveNetwork;
    for (const auto& [edgeId, chain] : discretizationResult.edgeIdToPointIndicesMap)
    {
        if (topology.getSeamCollection().isSeamTwin(edgeId))
            continue;
        const Geometry3D::IEdge3D* edge = geometry.getEdge(edgeId);
        if (edge && edge->isDegenerate())
            continue;
        curveNetwork[edgeId] = chain;
    }
    return curveNetwork;
}

// Inserts a new curve point into discretizationResult, directly after
// position positionInChain of edgeId's chain, and returns its point index.
//
// The index this returns is the currency every downstream consumer of
// DiscretizationResult3D deals in -- CurveSegmentBuilder's segments, the
// weights-by-index map subdivide() returns, Delaunay3D's point array -- so
// the one invariant here is that inserting a point must never move an
// existing one. That is why the point is APPENDED to the three parallel
// per-point arrays (points, edgeParameters and geometryIds, which grow in
// lockstep so index i keeps meaning the same point in all three), and only
// the owning edge's chain -- whose order along the curve is the one
// ordering that carries meaning -- has the new index spliced into its
// middle.
size_t insertCurvePoint(DiscretizationResult3D& discretizationResult,
                        const std::string& edgeId,
                        size_t positionInChain,
                        const Point3D& point,
                        double edgeParameter)
{
    const size_t newIndex = discretizationResult.points.size();
    discretizationResult.points.push_back(point);
    discretizationResult.edgeParameters.push_back({edgeParameter});
    discretizationResult.geometryIds.push_back({edgeId});

    std::vector<size_t>& chain = discretizationResult.edgeIdToPointIndicesMap.at(edgeId);
    chain.insert(chain.begin() + static_cast<std::ptrdiff_t>(positionInChain) + 1, newIndex);
    return newIndex;
}

// The edge parameter of the point at positionInChain. Same t-value
// convention CurveSegmentBuilder::build() uses: a chain's first and last
// positions are the edge's own parameter bounds (its corners), any
// interior position reads its own stored edge parameter.
double edgeParameterAt(const DiscretizationResult3D& discretizationResult,
                       const Geometry3D::IEdge3D& edge,
                       const std::vector<size_t>& chain,
                       size_t positionInChain)
{
    const auto [tMin, tMax] = edge.getParameterBounds();
    if (positionInChain == 0)
        return tMin;
    if (positionInChain == chain.size() - 1)
        return tMax;
    return discretizationResult.edgeParameters[chain[positionInChain]][0];
}

// Splits the segment (violation.nodeId1, violation.nodeId2) at its
// arc-length midpoint on the true curve, inserting the new point there
// (see insertCurvePoint() above). Returns false, and does nothing, if the
// edge can't be resolved or either resulting sub-segment would fall below
// minimumEdgeLength -- the caller treats that as "leave this violation
// permanently unresolved."
bool trySplit(const UnresolvedProtectionSegment& violation,
              DiscretizationResult3D& discretizationResult,
              const Geometry3D::GeometryCollection3D& geometry,
              double minimumEdgeLength)
{
    const Geometry3D::IEdge3D* edge = geometry.getEdge(violation.edgeId);
    if (!edge)
        return false;

    const auto& chain = discretizationResult.edgeIdToPointIndicesMap.at(violation.edgeId);
    const auto segmentStart = std::find(chain.begin(), chain.end(), violation.nodeId1);
    if (segmentStart == chain.end() || segmentStart + 1 == chain.end() || *(segmentStart + 1) != violation.nodeId2)
        return false;
    const size_t positionInChain = static_cast<size_t>(segmentStart - chain.begin());

    const double t1 = edgeParameterAt(discretizationResult, *edge, chain, positionInChain);
    const double t2 = edgeParameterAt(discretizationResult, *edge, chain, positionInChain + 1);
    const double tMid = edge->getParameterAtArcLengthFraction(t1, t2, 0.5);
    const Point3D newPoint = edge->getPoint(tMid);

    const double distanceToFirst = (newPoint - discretizationResult.points[chain[positionInChain]]).norm();
    const double distanceToSecond = (newPoint - discretizationResult.points[chain[positionInChain + 1]]).norm();
    if (distanceToFirst < minimumEdgeLength || distanceToSecond < minimumEdgeLength)
        return false;

    insertCurvePoint(discretizationResult, violation.edgeId, positionInChain, newPoint, tMid);
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

    AbandonedSegments abandonedSegments;

    std::unordered_map<size_t, double> weights;
    for (int iteration = 0; iteration < MAX_SUBDIVISION_ITERATIONS; ++iteration)
    {
        const auto curveNetwork = collectCurveNetwork(discretizationResult, topology, geometry);
        weights = CurveProtectionScheme::computeWeights(curveNetwork, cornerPointIndices, discretizationResult.points);
        const auto violations =
            CurveProtectionScheme::findUnresolvedSegments(curveNetwork, weights, discretizationResult.points);

        bool progressed = false;
        for (const auto& violation : violations)
        {
            if (abandonedSegments.contains(violation))
                continue;

            if (trySplit(violation, discretizationResult, geometry, minimumEdgeLength))
            {
                progressed = true;
                break; // the curve network gained a point; restart from a fresh computeWeights() next iteration
            }
            abandonedSegments.add(violation);
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
