#include "Meshing/Core/2D/General/MeshQueries2D.h"
#include "Meshing/Core/2D/General/ConstraintChecker2D.h"
#include "Meshing/Core/2D/General/ElementGeometry2D.h"
#include "Meshing/Core/2D/General/GeometryUtilities2D.h"
#include "Meshing/Core/2D/Periodic/PeriodicMeshData2D.h"
#include "Meshing/Data/2D/Node2D.h"
#include "Meshing/Data/Base/TriangleElement.h"
#include "Topology2D/Edge2D.h"
#include "Topology2D/Topology2D.h"
#include "spdlog/spdlog.h"
#include <algorithm>
#include <optional>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace Meshing
{

namespace
{

// Slack on the bounding box of a constraint segment when testing whether the insertion point lies on it.
constexpr double POINT_ON_SEGMENT_TOLERANCE = 1e-10;

// A cavity boundary edge whose signed area with the insertion point is below minus this would make an inverted triangle.
constexpr double INVERTED_TRIANGLE_AREA_TOLERANCE = 1e-10;

// The node-to-node and parameter tables a constrained segment's endpoints are read from.
struct SegmentEndpointLookup
{
    const std::map<size_t, size_t>& pointIndexToNodeIdMap;
    const std::vector<std::vector<double>>& tParameters;
    const std::vector<std::vector<std::string>>& geometryIds;
};

bool isInCircumcircleConflict(const MeshData2D& meshData,
                              const ElementGeometry2D& geometry,
                              size_t triangleId,
                              const TriangleElement& triangle,
                              const Point2D& point);
bool isVisibleFromTriangleInterior(const MeshData2D& meshData,
                                   const TriangleElement& triangle,
                                   const Point2D& centroid,
                                   const Point2D& point);
bool isPointOnSegment(const Point2D& segmentStart, const Point2D& segmentEnd, const Point2D& point);
void removeStarShapednessViolations(const MeshData2D& meshData, const Point2D& point, std::vector<size_t>& conflicting);
std::optional<size_t> findInvertingCavityTriangle(const MeshData2D& meshData,
                                                  const Point2D& point,
                                                  const std::vector<size_t>& conflicting);
std::unordered_set<std::string> collectBoundaryEdgeIds(const Topology2D::Topology2D& topology);
CurveSegment createSegment(const SegmentEndpointLookup& endpointLookup,
                           size_t startPointIndex,
                           size_t endPointIndex,
                           const std::string& edgeId,
                           ConstraintRole role);
double findParameterOnEdge(const SegmentEndpointLookup& endpointLookup, size_t pointIndex, const std::string& edgeId);
const char* describeConstraintRole(ConstraintRole role);
std::optional<size_t> findInteriorSeedTriangle(const MeshQueries2D& queries,
                                               const MeshData2D& meshData,
                                               const ElementGeometry2D& geometry);
std::unordered_set<size_t> floodFillToBoundary(const MeshQueries2D& queries,
                                               const MeshData2D& meshData,
                                               const MeshQueries2D::EdgeToTrianglesMap& edgeToTriangles,
                                               size_t seedTriangleId);

} // namespace

MeshQueries2D::MeshQueries2D(const MeshData2D& meshData) :
    MeshQueries2D(meshData, nullptr)
{
}

MeshQueries2D::MeshQueries2D(const MeshData2D& meshData, PeriodicMeshData2D* periodicData) :
    meshData_(meshData),
    periodicData_(periodicData)
{
}

std::vector<size_t> MeshQueries2D::findConflictingTriangles(const Point2D& point) const
{
    std::vector<size_t> conflicting;

    ElementGeometry2D geometry(meshData_, periodicData_);

    for (const auto& [id, element] : meshData_.getElements())
    {
        auto triangle = dynamic_cast<const TriangleElement*>(element.get());
        if (triangle == nullptr)
        {
            SPDLOG_WARN("MeshQueries2D: Skipping non-triangle element {}", id);
            continue;
        }

        if (!isInCircumcircleConflict(meshData_, geometry, id, *triangle, point)) continue;

        // Visibility check per constrained Delaunay definition (Shewchuk, p.2):
        // A triangle is in the conflict set only if the insertion point is visible
        // from the triangle's interior, i.e. the line from point to centroid does
        // not properly cross any constrained segment.
        if (isVisibleFromTriangleInterior(meshData_, *triangle, geometry.computeCentroid(id), point)) conflicting.push_back(id);
    }

    removeStarShapednessViolations(meshData_, point, conflicting);

    return conflicting;
}

std::vector<std::array<size_t, 2>> MeshQueries2D::findCavityBoundary(const std::vector<size_t>& conflictingIndices) const
{
    // Count how many times each edge appears
    std::map<EdgeKey, int> edgeCount;
    std::map<EdgeKey, std::array<size_t, 2>> edgeLookup;

    for (size_t index : conflictingIndices)
    {
        auto& triangle = dynamic_cast<const TriangleElement&>(*meshData_.getElement(index));
        for (size_t edgeIndex = 0; edgeIndex < 3; ++edgeIndex)
        {
            auto edge = triangle.getEdge(edgeIndex);
            auto key = makeEdgeKey(edge[0], edge[1]);
            edgeCount[key]++;
            edgeLookup[key] = edge; // Store original order
        }
    }

    // Boundary edges appear exactly once
    std::vector<std::array<size_t, 2>> boundary;
    for (const auto& [key, count] : edgeCount)
    {
        if (count == 1) boundary.push_back(edgeLookup.at(key));
    }

    return boundary;
}

std::vector<size_t> MeshQueries2D::findIntersectingTriangles(size_t nodeId1, size_t nodeId2) const
{
    std::vector<size_t> result;

    auto& constraintStart = meshData_.getNode(nodeId1)->getCoordinates();
    auto& constraintEnd = meshData_.getNode(nodeId2)->getCoordinates();

    for (const auto& [elementId, element] : meshData_.getElements())
    {
        auto triangle = dynamic_cast<const TriangleElement*>(element.get());
        if (!triangle)
        {
            spdlog::warn("MeshQueries2D: Skipping non-triangle element {}", elementId);
            continue;
        }

        if (triangle->hasNode(nodeId1) && triangle->hasNode(nodeId2)) continue;

        bool intersects = false;
        for (size_t edgeIndex = 0; edgeIndex < 3; ++edgeIndex)
        {
            auto edge = triangle->getEdge(edgeIndex);

            // Skip edges that share a node with the constraint edge
            if (edge[0] == nodeId1 || edge[0] == nodeId2 ||
                edge[1] == nodeId1 || edge[1] == nodeId2)
            {
                continue;
            }

            auto& edgeStart = meshData_.getNode(edge[0])->getCoordinates();
            auto& edgeEnd = meshData_.getNode(edge[1])->getCoordinates();

            if (GeometryUtilities2D::segmentsIntersect(constraintStart, constraintEnd, edgeStart, edgeEnd))
            {
                intersects = true;
                break;
            }
        }

        if (intersects) result.push_back(elementId);
    }

    return result;
}

CurveSegmentManager MeshQueries2D::extractConstrainedEdges(
    const Topology2D::Topology2D& topology,
    const std::map<std::string, size_t>& cornerIdToPointIndexMap,
    const std::map<size_t, size_t>& pointIndexToNodeIdMap,
    const std::map<std::string, std::vector<size_t>>& edgeIdToPointIndicesMap,
    const std::vector<std::vector<double>>& tParameters,
    const std::vector<std::vector<std::string>>& geometryIds) const
{
    CurveSegmentManager manager;
    SegmentEndpointLookup endpointLookup{pointIndexToNodeIdMap, tParameters, geometryIds};
    auto boundaryEdgeIds = collectBoundaryEdgeIds(topology);

    for (const auto& edgeId : topology.getAllEdgeIds())
    {
        ConstraintRole role = boundaryEdgeIds.count(edgeId) ? ConstraintRole::Boundary : ConstraintRole::Interior;

        auto edgePointsIterator = edgeIdToPointIndicesMap.find(edgeId);
        if (edgePointsIterator == edgeIdToPointIndicesMap.end() || edgePointsIterator->second.size() < 2)
        {
            auto& edge = topology.getEdge(edgeId);
            size_t startPointIndex = cornerIdToPointIndexMap.at(edge.getStartCornerId());
            size_t endPointIndex = cornerIdToPointIndexMap.at(edge.getEndCornerId());
            auto segment = createSegment(endpointLookup, startPointIndex, endPointIndex, edgeId, role);
            manager.addSegment(segment);

            spdlog::debug("Edge {}: Node IDs ({}, {}), t=[{},{}], role: {}",
                          edgeId, segment.nodeId1, segment.nodeId2, segment.tStart, segment.tEnd,
                          describeConstraintRole(role));
            continue;
        }

        auto& pointIndices = edgePointsIterator->second;
        for (size_t segmentIndex = 0; segmentIndex < pointIndices.size() - 1; ++segmentIndex)
        {
            auto segment = createSegment(endpointLookup, pointIndices[segmentIndex], pointIndices[segmentIndex + 1], edgeId, role);
            manager.addSegment(segment);

            spdlog::debug("Edge {} segment {}: Node IDs ({}, {}), t=[{},{}], role: {}",
                          edgeId, segmentIndex, segment.nodeId1, segment.nodeId2, segment.tStart, segment.tEnd,
                          describeConstraintRole(role));
        }
    }

    return manager;
}

std::vector<CurveSegment> MeshQueries2D::findEncroachedSegments() const
{
    std::vector<CurveSegment> encroached;

    ConstraintChecker2D constraintChecker(meshData_, periodicData_);

    for (const auto& [segmentId, segment] : meshData_.getCurveSegmentManager().getAllSegments())
    {
        for (const auto& [nodeId, node] : meshData_.getNodes())
        {
            if (nodeId == segment.nodeId1 || nodeId == segment.nodeId2) continue;

            auto& point = node->getCoordinates();

            // Per Ruppert: only count encroachment if the vertex is visible
            // from the subsegment interior (visibility obstructed by other segments)
            if (constraintChecker.isSegmentEncroached(segment, point) && isPointVisibleFromSegment(point, segment))
            {
                encroached.push_back(segment);
                break;
            }
        }
    }

    return encroached;
}

std::vector<CurveSegment> MeshQueries2D::findSegmentsEncroachedByPoint(
    const Point2D& point) const
{
    std::vector<CurveSegment> encroached;

    ConstraintChecker2D constraintChecker(meshData_, periodicData_);

    for (const auto& [segmentId, segment] : meshData_.getCurveSegmentManager().getAllSegments())
    {
        // Per Ruppert: only count encroachment if the point is visible
        // from the subsegment interior
        if (constraintChecker.isSegmentEncroached(segment, point) && isPointVisibleFromSegment(point, segment)) encroached.push_back(segment);
    }

    return encroached;
}

bool MeshQueries2D::isPointVisibleFromSegment(const Point2D& point, const CurveSegment& segment) const
{
    auto& segmentStart = meshData_.getNode(segment.nodeId1)->getCoordinates();
    auto& segmentEnd = meshData_.getNode(segment.nodeId2)->getCoordinates();
    Point2D midpoint = (segmentStart + segmentEnd) * 0.5;

    // Check if the line from point to midpoint properly crosses any other
    // constrained segment. "Properly" means the segments cross in their interiors,
    // not just touch at endpoints.
    for (const auto& [otherId, other] : meshData_.getCurveSegmentManager().getAllSegments())
    {
        if (other.nodeId1 == segment.nodeId1 && other.nodeId2 == segment.nodeId2) continue;
        if (other.nodeId1 == segment.nodeId2 && other.nodeId2 == segment.nodeId1) continue;

        auto& otherStart = meshData_.getNode(other.nodeId1)->getCoordinates();
        auto& otherEnd = meshData_.getNode(other.nodeId2)->getCoordinates();

        // Segments sharing an endpoint with the query segment or touching at the
        // point itself should not block visibility.
        int otherStartSide = GeometryUtilities2D::computeOrientationSign(point, midpoint, otherStart);
        int otherEndSide = GeometryUtilities2D::computeOrientationSign(point, midpoint, otherEnd);
        int pointSide = GeometryUtilities2D::computeOrientationSign(otherStart, otherEnd, point);
        int midpointSide = GeometryUtilities2D::computeOrientationSign(otherStart, otherEnd, midpoint);

        // Proper crossing: both endpoints of each segment are strictly on opposite
        // sides of the other segment
        if (otherStartSide != 0 && otherEndSide != 0 && otherStartSide != otherEndSide &&
            pointSide != 0 && midpointSide != 0 && pointSide != midpointSide)
        {
            return false;
        }
    }

    return true;
}

std::vector<size_t> MeshQueries2D::findTrianglesAdjacentToEdge(size_t nodeId1, size_t nodeId2) const
{
    std::vector<size_t> adjacent;
    auto edgeKey = makeEdgeKey(nodeId1, nodeId2);

    for (const auto& [elementId, element] : meshData_.getElements())
    {
        auto triangle = dynamic_cast<const TriangleElement*>(element.get());
        if (!triangle) continue;

        for (size_t edgeIndex = 0; edgeIndex < 3; ++edgeIndex)
        {
            auto edge = triangle->getEdge(edgeIndex);
            if (makeEdgeKey(edge[0], edge[1]) == edgeKey)
            {
                adjacent.push_back(elementId);
                break;
            }
        }
    }

    return adjacent;
}

bool MeshQueries2D::isPointInsideDomain(const Point2D& point) const
{
    int crossings = 0;
    double pointX = point.x();
    double pointY = point.y();

    for (const auto& [segmentId, segment] : meshData_.getCurveSegmentManager().getAllSegments())
    {
        if (segment.role != ConstraintRole::Boundary) continue;

        auto startNode = meshData_.getNode(segment.nodeId1);
        auto endNode = meshData_.getNode(segment.nodeId2);
        if (!startNode || !endNode) continue;

        auto& start = startNode->getCoordinates();
        auto& end = endNode->getCoordinates();
        double startY = start.y();
        double endY = end.y();

        // Check if the segment straddles the ray's Y coordinate
        if ((startY <= pointY && endY > pointY) || (endY <= pointY && startY > pointY))
        {
            double parameter = (pointY - startY) / (endY - startY);
            double intersectionX = start.x() + parameter * (end.x() - start.x());

            if (pointX < intersectionX) crossings++;
        }
    }

    return (crossings % 2) == 1;
}

std::unordered_set<size_t> MeshQueries2D::classifyTrianglesInteriorExterior() const
{
    if (meshData_.getElements().empty())
    {
        spdlog::warn("classifyTrianglesInteriorExterior: No triangles in mesh");
        return {};
    }

    if (meshData_.getCurveSegmentManager().empty())
    {
        spdlog::warn("classifyTrianglesInteriorExterior: No constraint edges, skipping classification");
        return {};
    }

    ElementGeometry2D geometry(meshData_, periodicData_);
    auto seedTriangleId = findInteriorSeedTriangle(*this, meshData_, geometry);
    if (!seedTriangleId)
    {
        spdlog::error("classifyTrianglesInteriorExterior: Ray casting found no interior triangle");
        return {};
    }

    spdlog::debug("classifyTrianglesInteriorExterior: Ray casting found interior seed triangle {}", *seedTriangleId);

    auto insideTriangles = floodFillToBoundary(*this, meshData_, buildEdgeToTrianglesMap(), *seedTriangleId);

    spdlog::debug("classifyTrianglesInteriorExterior: Flood fill found {} interior triangles", insideTriangles.size());

    return insideTriangles;
}

MeshQueries2D::EdgeToTrianglesMap MeshQueries2D::buildEdgeToTrianglesMap() const
{
    EdgeToTrianglesMap edgeToTriangles;

    for (const auto& [elementId, element] : meshData_.getElements())
    {
        auto triangle = dynamic_cast<const TriangleElement*>(element.get());
        if (!triangle) continue;

        for (size_t edgeIndex = 0; edgeIndex < 3; ++edgeIndex)
        {
            auto edge = triangle->getEdge(edgeIndex);
            EdgeKey key = makeEdgeKey(edge[0], edge[1]);
            edgeToTriangles[key].push_back(elementId);
        }
    }

    return edgeToTriangles;
}

bool MeshQueries2D::isBoundaryConstraintEdge(const std::array<size_t, 2>& edge) const
{
    for (const auto& [segmentId, segment] : meshData_.getCurveSegmentManager().getAllSegments())
    {
        if (segment.role != ConstraintRole::Boundary) continue;

        if ((segment.nodeId1 == edge[0] && segment.nodeId2 == edge[1]) ||
            (segment.nodeId1 == edge[1] && segment.nodeId2 == edge[0]))
        {
            return true;
        }
    }

    return false;
}

namespace
{

bool isInCircumcircleConflict(const MeshData2D& meshData,
                              const ElementGeometry2D& geometry,
                              size_t triangleId,
                              const TriangleElement& triangle,
                              const Point2D& point)
{
    auto circle = geometry.computeCircumcircle(triangleId);
    if (circle) return GeometryUtilities2D::isPointInsideCircle(*circle, point);

    // Degenerate triangle: circumcircle is undefined (nearly collinear vertices).
    // Fall back to direct containment — a point on the triangle boundary
    // (e.g. the midpoint of a segment being split) must still be included.
    auto& ids = triangle.getNodeIdArray();
    auto& vertex0 = meshData.getNode(ids[0])->getCoordinates();
    auto& vertex1 = meshData.getNode(ids[1])->getCoordinates();
    auto& vertex2 = meshData.getNode(ids[2])->getCoordinates();
    return GeometryUtilities2D::isPointInsideOrOnTriangle(point, vertex0, vertex1, vertex2);
}

bool isVisibleFromTriangleInterior(const MeshData2D& meshData,
                                   const TriangleElement& triangle,
                                   const Point2D& centroid,
                                   const Point2D& point)
{
    auto& nodeIds = triangle.getNodeIdArray();
    for (const auto& [segmentId, segment] : meshData.getCurveSegmentManager().getAllSegments())
    {
        // Skip segments that share a node with this triangle
        if (segment.nodeId1 == nodeIds[0] || segment.nodeId1 == nodeIds[1] ||
            segment.nodeId1 == nodeIds[2] || segment.nodeId2 == nodeIds[0] ||
            segment.nodeId2 == nodeIds[1] || segment.nodeId2 == nodeIds[2])
        {
            continue;
        }

        auto& segmentStart = meshData.getNode(segment.nodeId1)->getCoordinates();
        auto& segmentEnd = meshData.getNode(segment.nodeId2)->getCoordinates();

        // If the insertion point lies on this constraint segment, the line from
        // point to centroid starts on the segment — that is not a proper crossing
        // and cannot block visibility (e.g. when splitting a seam segment at its midpoint).
        if (isPointOnSegment(segmentStart, segmentEnd, point)) continue;

        if (GeometryUtilities2D::segmentsIntersect(point, centroid, segmentStart, segmentEnd)) return false;
    }

    return true;
}

bool isPointOnSegment(const Point2D& segmentStart, const Point2D& segmentEnd, const Point2D& point)
{
    return GeometryUtilities2D::computeOrientationSign(segmentStart, segmentEnd, point) == 0 &&
           point.x() >= std::min(segmentStart.x(), segmentEnd.x()) - POINT_ON_SEGMENT_TOLERANCE &&
           point.x() <= std::max(segmentStart.x(), segmentEnd.x()) + POINT_ON_SEGMENT_TOLERANCE &&
           point.y() >= std::min(segmentStart.y(), segmentEnd.y()) - POINT_ON_SEGMENT_TOLERANCE &&
           point.y() <= std::max(segmentStart.y(), segmentEnd.y()) + POINT_ON_SEGMENT_TOLERANCE;
}

// Star-shapedness verification: remove cavity triangles whose boundary edges
// would produce inverted triangles when connected to the insertion point
// (guards against degenerate cases near constraint boundaries).
void removeStarShapednessViolations(const MeshData2D& meshData, const Point2D& point, std::vector<size_t>& conflicting)
{
    while (auto invertingTriangleId = findInvertingCavityTriangle(meshData, point, conflicting))
    {
        std::erase(conflicting, *invertingTriangleId);
    }
}

std::optional<size_t> findInvertingCavityTriangle(const MeshData2D& meshData,
                                                  const Point2D& point,
                                                  const std::vector<size_t>& conflicting)
{
    std::unordered_map<MeshQueries2D::EdgeKey, size_t, MeshQueries2D::EdgeKeyHash> edgeCount;
    std::unordered_map<MeshQueries2D::EdgeKey, std::array<size_t, 2>, MeshQueries2D::EdgeKeyHash> edgeOriginal;
    std::unordered_map<MeshQueries2D::EdgeKey, size_t, MeshQueries2D::EdgeKeyHash> edgeOwner;

    for (size_t id : conflicting)
    {
        auto triangle = dynamic_cast<const TriangleElement*>(meshData.getElement(id));
        for (size_t edgeIndex = 0; edgeIndex < 3; ++edgeIndex)
        {
            auto edge = triangle->getEdge(edgeIndex);
            MeshQueries2D::EdgeKey key = MeshQueries2D::makeEdgeKey(edge[0], edge[1]);
            edgeCount[key]++;
            edgeOriginal[key] = edge;
            edgeOwner[key] = id;
        }
    }

    for (const auto& [key, count] : edgeCount)
    {
        if (count != 1) continue;

        auto& edge = edgeOriginal[key];
        auto& edgeStart = meshData.getNode(edge[0])->getCoordinates();
        auto& edgeEnd = meshData.getNode(edge[1])->getCoordinates();
        double area = GeometryUtilities2D::computeSignedArea(point, edgeStart, edgeEnd);

        if (area < -INVERTED_TRIANGLE_AREA_TOLERANCE) return edgeOwner[key];
    }

    return std::nullopt;
}

// The edges of the outer loop and of every hole loop.
std::unordered_set<std::string> collectBoundaryEdgeIds(const Topology2D::Topology2D& topology)
{
    std::unordered_set<std::string> boundaryEdgeIds;
    for (const auto& edgeId : topology.getOuterEdgeLoop())
    {
        boundaryEdgeIds.insert(edgeId);
    }
    for (const auto& holeLoop : topology.getHoleEdgeLoops())
    {
        for (const auto& edgeId : holeLoop)
        {
            boundaryEdgeIds.insert(edgeId);
        }
    }
    return boundaryEdgeIds;
}

CurveSegment createSegment(const SegmentEndpointLookup& endpointLookup,
                           size_t startPointIndex,
                           size_t endPointIndex,
                           const std::string& edgeId,
                           ConstraintRole role)
{
    size_t startNodeId = endpointLookup.pointIndexToNodeIdMap.at(startPointIndex);
    size_t endNodeId = endpointLookup.pointIndexToNodeIdMap.at(endPointIndex);
    double tStart = findParameterOnEdge(endpointLookup, startPointIndex, edgeId);
    double tEnd = findParameterOnEdge(endpointLookup, endPointIndex, edgeId);
    return CurveSegment{startNodeId, endNodeId, edgeId, tStart, tEnd, role};
}

// The point's parameter on the given edge, or 0 when the point carries none for it.
double findParameterOnEdge(const SegmentEndpointLookup& endpointLookup, size_t pointIndex, const std::string& edgeId)
{
    if (pointIndex >= endpointLookup.tParameters.size() || pointIndex >= endpointLookup.geometryIds.size()) return 0.0;

    auto& pointGeometryIds = endpointLookup.geometryIds[pointIndex];
    auto& pointTParameters = endpointLookup.tParameters[pointIndex];
    for (size_t geometryIndex = 0; geometryIndex < pointGeometryIds.size(); ++geometryIndex)
    {
        if (pointGeometryIds[geometryIndex] == edgeId) return pointTParameters[geometryIndex];
    }
    return 0.0;
}

const char* describeConstraintRole(ConstraintRole role)
{
    return role == ConstraintRole::Boundary ? "boundary" : "interior";
}

// The first triangle, in element order, whose centroid ray casting puts inside the domain.
std::optional<size_t> findInteriorSeedTriangle(const MeshQueries2D& queries,
                                               const MeshData2D& meshData,
                                               const ElementGeometry2D& geometry)
{
    for (const auto& [elementId, element] : meshData.getElements())
    {
        auto triangle = dynamic_cast<const TriangleElement*>(element.get());
        if (!triangle) continue;

        Point2D centroid = geometry.computeCentroid(*triangle);
        if (queries.isPointInsideDomain(centroid)) return elementId;
    }
    return std::nullopt;
}

// Every triangle reachable from the seed without crossing a boundary constraint edge.
std::unordered_set<size_t> floodFillToBoundary(const MeshQueries2D& queries,
                                               const MeshData2D& meshData,
                                               const MeshQueries2D::EdgeToTrianglesMap& edgeToTriangles,
                                               size_t seedTriangleId)
{
    std::unordered_set<size_t> insideTriangles;
    std::queue<size_t> queue;
    queue.push(seedTriangleId);
    insideTriangles.insert(seedTriangleId);

    while (!queue.empty())
    {
        size_t currentTriangleId = queue.front();
        queue.pop();

        auto element = meshData.getElement(currentTriangleId);
        if (!element) continue;

        auto triangle = dynamic_cast<const TriangleElement*>(element);
        if (!triangle) continue;

        for (size_t edgeIndex = 0; edgeIndex < 3; ++edgeIndex)
        {
            auto edge = triangle->getEdge(edgeIndex);
            if (queries.isBoundaryConstraintEdge(edge)) continue;

            auto edgeKey = MeshQueries2D::makeEdgeKey(edge[0], edge[1]);
            auto iterator = edgeToTriangles.find(edgeKey);
            if (iterator == edgeToTriangles.end()) continue;

            for (size_t neighborId : iterator->second)
            {
                if (neighborId != currentTriangleId &&
                    !insideTriangles.contains(neighborId))
                {
                    insideTriangles.insert(neighborId);
                    queue.push(neighborId);
                }
            }
        }
    }

    return insideTriangles;
}

} // namespace

} // namespace Meshing
