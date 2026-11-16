#include "Meshing/Core/2D/General/MeshOperations2D.h"
#include "Common/Exceptions/MeshException.h"
#include "Geometry/2D/Base/IEdge2D.h"
#include "Meshing/Core/2D/General/ElementGeometry2D.h"
#include "Meshing/Core/2D/General/GeometryUtilities2D.h"
#include "Meshing/Core/2D/Periodic/PeriodicMeshData2D.h"
#include "Meshing/Data/2D/MeshMutator2D.h"
#include "Meshing/Data/2D/Node2D.h"
#include "Meshing/Data/Base/TriangleElement.h"
#include "spdlog/spdlog.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

namespace Meshing
{

namespace
{

// Below this absolute signed area a triangle counts as degenerate and is not created.
constexpr double MIN_TRIANGLE_AREA = 1e-10;
constexpr double MIN_FLIP_AREA = 1e-10;
constexpr double NODE_COINCIDENCE_TOLERANCE = 1e-10;
constexpr double MIN_CHORD_LENGTH_SQUARED = 1e-20;
constexpr double MAX_CHORD_DEVIATION_SQUARED = 1e-12;

using EdgeKey = MeshQueries2D::EdgeKey;
using EdgeKeySet = std::unordered_set<EdgeKey, MeshQueries2D::EdgeKeyHash>;

// Offsets of a boundary edge's two nodes, keyed and ordered by node id: {min node, max node}.
using EdgeOffsetMap = std::unordered_map<EdgeKey, std::array<PeriodicOffset, 2>, MeshQueries2D::EdgeKeyHash>;

// The two triangles that replace a flipped edge's pair, wound counter-clockwise.
struct EdgeFlip
{
    NodeId oppositeNode0;
    NodeId oppositeNode1;
    std::array<NodeId, 3> triangle0;
    std::array<NodeId, 3> triangle1;
};

EdgeOffsetMap collectBoundaryEdgeOffsets(const MeshData2D& meshData,
                                         const PeriodicOffsetTable2D& offsetTable,
                                         const std::vector<ElementId>& triangleIds);
std::array<PeriodicOffset, 2> findEdgeOffsets(const EdgeOffsetMap& edgeOffsets, const std::array<NodeId, 2>& edge);
std::vector<NodeId> traceBoundaryLoop(const std::vector<std::array<NodeId, 2>>& boundary, NodeId startNodeId);
void triangulatePolygon(const MeshData2D& meshData, MeshMutator2D& mutator, std::vector<NodeId> polygon);
void orientCounterClockwise(const MeshData2D& meshData, std::vector<NodeId>& polygon);
bool clipEar(const MeshData2D& meshData, MeshMutator2D& mutator, std::vector<NodeId>& polygon);
bool isEar(const MeshData2D& meshData, const std::vector<NodeId>& polygon, size_t previous, size_t current, size_t next);
void addFinalTriangle(const MeshData2D& meshData, MeshMutator2D& mutator, const std::vector<NodeId>& polygon);
EdgeKeySet collectConstrainedEdgeKeys(const MeshData2D& meshData);
std::vector<EdgeKey> collectUnconstrainedEdges(const MeshData2D& meshData,
                                               const std::vector<ElementId>& triangleIds,
                                               const EdgeKeySet& constrainedEdgeKeys);
std::optional<EdgeFlip> findDelaunayFlip(const MeshData2D& meshData,
                                         const ElementGeometry2D& elementGeometry,
                                         NodeId edgeNode0,
                                         NodeId edgeNode1,
                                         const std::vector<ElementId>& adjacent);
std::optional<NodeId> findCoincidentNode(const MeshData2D& meshData, const Point2D& point);
bool isOnChord(const Point2D& chordStart, const Point2D& chordEnd, const Point2D& point);

} // namespace

MeshOperations2D::MeshOperations2D(MeshData2D& meshData) :
    MeshOperations2D(meshData, nullptr)
{
}

MeshOperations2D::MeshOperations2D(MeshData2D& meshData, PeriodicMeshData2D* periodicData) :
    meshData_(meshData),
    periodicData_(periodicData),
    queries_(meshData, periodicData),
    mutator_(std::make_unique<MeshMutator2D>(meshData))
{
}

NodeId MeshOperations2D::insertVertexBowyerWatson(const Point2D& point,
                                                  const std::vector<std::string>& edgeIds)
{
    std::vector<ElementId> conflicting = queries_.findConflictingTriangles(point);

    if (conflicting.empty())
    {
        SPDLOG_DEBUG("MeshOperations2D: No conflicting triangles found for point ({}, {}) — blocked by constraints",
                     point.x(), point.y());
        OPENLOOM_THROW_CODE(OpenLoom::MeshException,
                            OpenLoom::MeshException::ErrorCode::INVALID_OPERATION,
                            "No conflicting triangles found for point (" + std::to_string(point.x()) + ", " + std::to_string(point.y()) + ")");
    }

    std::vector<std::array<NodeId, 2>> boundary = queries_.findCavityBoundary(conflicting);

    // In a periodic triangulation the new triangle {newVertex, e0, e1} inherits the offsets
    // the removed triangle assigned to e0 and e1, so they are read before the removal.
    EdgeOffsetMap boundaryEdgeOffsets;
    if (periodicData_)
    {
        auto& offsetTable = periodicData_->getOffsetTable();
        boundaryEdgeOffsets = collectBoundaryEdgeOffsets(meshData_, offsetTable, conflicting);
        for (ElementId conflictId : conflicting)
            offsetTable.removeOffsets(conflictId);
    }

    for (auto conflictingElementId : conflicting)
    {
        mutator_->removeElement(conflictingElementId);
    }

    NodeId newVertex = edgeIds.empty() ? mutator_->addNode(point) : mutator_->addBoundaryNode(point, edgeIds);
    Point2D newPoint = meshData_.getNode(newVertex)->getCoordinates();

    for (const auto& edge : boundary)
    {
        std::array<PeriodicOffset, 2> edgeOffsets = {};
        Point2D edgePoint0 = meshData_.getNode(edge[0])->getCoordinates();
        Point2D edgePoint1 = meshData_.getNode(edge[1])->getCoordinates();
        if (periodicData_)
        {
            edgeOffsets = findEdgeOffsets(boundaryEdgeOffsets, edge);
            edgePoint0 = periodicData_->applyOffset(edgePoint0, edgeOffsets[0]);
            edgePoint1 = periodicData_->applyOffset(edgePoint1, edgeOffsets[1]);
        }

        double area = GeometryUtilities2D::computeSignedArea(newPoint, edgePoint0, edgePoint1);

        if (std::abs(area) < MIN_TRIANGLE_AREA)
        {
            spdlog::debug("Skipping degenerate triangle in Bowyer-Watson: ({}, {}, {})",
                          newVertex.value(), edge[0].value(), edge[1].value());
            continue;
        }

        ElementId newTriangleId = mutator_->addElement(
            std::make_unique<TriangleElement>(std::array<NodeId, 3>{newVertex, edge[0], edge[1]}));

        if (periodicData_)
        {
            periodicData_->getOffsetTable().setOffsets(newTriangleId, {PeriodicOffset{}, edgeOffsets[0], edgeOffsets[1]});
        }
    }

    return newVertex;
}

bool MeshOperations2D::removeTrianglesContainingNode(NodeId nodeId)
{
    std::vector<ElementId> elementsToRemove;
    for (const auto& [id, element] : meshData_.getElements())
    {
        auto triangle = dynamic_cast<const TriangleElement*>(element.get());
        if (triangle && triangle->hasNode(nodeId))
        {
            elementsToRemove.push_back(id);
        }
    }

    for (ElementId id : elementsToRemove)
    {
        mutator_->removeElement(id);
    }

    return !elementsToRemove.empty();
}

bool MeshOperations2D::enforceEdge(NodeId nodeId1, NodeId nodeId2)
{
    auto existingTriangles = queries_.findTrianglesAdjacentToEdge(nodeId1, nodeId2);
    if (!existingTriangles.empty())
    {
        spdlog::debug("Edge ({}, {}) already exists in mesh (tri {})", nodeId1.value(), nodeId2.value(), existingTriangles[0].value());
        return true;
    }

    auto intersectingTriangles = queries_.findIntersectingTriangles(nodeId1, nodeId2);

    if (intersectingTriangles.empty())
    {
        spdlog::warn("Edge ({}, {}) does not exist but no intersecting triangles found", nodeId1.value(), nodeId2.value());
        return true; // No triangles to remove, edge might already be enforced
    }

    std::vector<std::array<NodeId, 2>> boundary = queries_.findCavityBoundary(intersectingTriangles);

    for (auto index : intersectingTriangles)
    {
        mutator_->removeElement(index);
    }

    // Retriangulate the cavity as two sub-polygons, one on each side of the constraint edge:
    // the boundary loop runs nodeId1 -> ... -> nodeId2 -> ... -> back to nodeId1.
    auto boundaryLoop = traceBoundaryLoop(boundary, nodeId1);

    auto it = std::find(boundaryLoop.begin(), boundaryLoop.end(), nodeId2);
    if (it == boundaryLoop.end())
    {
        spdlog::warn("enforceEdge: nodeId2 ({}) not found in traced boundary for edge ({}, {})",
                     nodeId2.value(), nodeId1.value(), nodeId2.value());
        return false;
    }

    std::vector<NodeId> leftPolygon(boundaryLoop.begin(), it + 1);
    std::vector<NodeId> rightPolygon(it, boundaryLoop.end());
    rightPolygon.push_back(nodeId1);

    triangulatePolygon(meshData_, *mutator_, std::move(leftPolygon));
    triangulatePolygon(meshData_, *mutator_, std::move(rightPolygon));

    return true;
}

std::vector<ElementId> MeshOperations2D::splitTrianglesAtEdge(NodeId edgeNode1, NodeId edgeNode2, NodeId midNodeId)
{
    auto adjacentTriangles = queries_.findTrianglesAdjacentToEdge(edgeNode1, edgeNode2);
    std::vector<ElementId> newTriangleIds;

    for (ElementId triangleId : adjacentTriangles)
    {
        auto triangle = dynamic_cast<const TriangleElement*>(meshData_.getElement(triangleId));
        if (!triangle)
            continue;

        // Skip hole-interior triangles. When a curved constraint (arc) is split,
        // the arc midpoint lies on the hole boundary and can fall geometrically
        // outside a bridging triangle, producing a CW-wound sub-triangle.
        // These triangles will be removed by the next classifyAndRemoveExteriorTriangles
        // call — skipping them here is correct.
        auto& nodeIds = triangle->getNodeIdArray();
        auto& point0 = meshData_.getNode(nodeIds[0])->getCoordinates();
        auto& point1 = meshData_.getNode(nodeIds[1])->getCoordinates();
        auto& point2 = meshData_.getNode(nodeIds[2])->getCoordinates();
        if (!queries_.isPointInsideDomain((point0 + point1 + point2) / 3.0))
            continue;

        NodeId opposite = triangle->getOppositeNode(edgeNode1, edgeNode2);

        // Find the actual direction of the edge in this triangle's winding order.
        // Each of the two adjacent triangles sees the shared edge in opposite directions,
        // so we must use the triangle's stored order to preserve CCW orientation.
        NodeId first = edgeNode1;
        NodeId second = edgeNode2;
        for (size_t i = 0; i < 3; ++i)
        {
            auto edge = triangle->getEdge(i);
            if (edge[0] == edgeNode2 && edge[1] == edgeNode1)
            {
                std::swap(first, second);
                break;
            }
        }

        mutator_->removeElement(triangleId);

        ElementId triangleId0 = mutator_->addElement(
            std::make_unique<TriangleElement>(std::array<NodeId, 3>{first, midNodeId, opposite}));
        ElementId triangleId1 = mutator_->addElement(
            std::make_unique<TriangleElement>(std::array<NodeId, 3>{midNodeId, second, opposite}));

        newTriangleIds.push_back(triangleId0);
        newTriangleIds.push_back(triangleId1);
    }

    return newTriangleIds;
}

void MeshOperations2D::lawsonFlip(const std::vector<ElementId>& newTriangleIds)
{
    auto constrainedEdgeKeys = collectConstrainedEdgeKeys(meshData_);
    auto edgeStack = collectUnconstrainedEdges(meshData_, newTriangleIds, constrainedEdgeKeys);

    ElementGeometry2D elementGeometry(meshData_);

    while (!edgeStack.empty())
    {
        auto [edgeNode0, edgeNode1] = edgeStack.back();
        edgeStack.pop_back();

        auto adjacent = queries_.findTrianglesAdjacentToEdge(edgeNode0, edgeNode1);
        if (adjacent.size() != 2)
            continue;

        auto flip = findDelaunayFlip(meshData_, elementGeometry, edgeNode0, edgeNode1, adjacent);
        if (!flip)
            continue;

        if (periodicData_)
        {
            auto& offsetTable = periodicData_->getOffsetTable();
            offsetTable.removeOffsets(adjacent[0]);
            offsetTable.removeOffsets(adjacent[1]);
        }

        mutator_->removeElement(adjacent[0]);
        mutator_->removeElement(adjacent[1]);

        ElementId flippedTriangleId0 = mutator_->addElement(std::make_unique<TriangleElement>(flip->triangle0));
        ElementId flippedTriangleId1 = mutator_->addElement(std::make_unique<TriangleElement>(flip->triangle1));

        // Register zero offsets for the flipped triangles so they use canonical
        // coordinates until a proper offset propagation pass is done in Phase 3.
        if (periodicData_)
        {
            auto& offsetTable = periodicData_->getOffsetTable();
            offsetTable.setOffsets(flippedTriangleId0, {PeriodicOffset{}, PeriodicOffset{}, PeriodicOffset{}});
            offsetTable.setOffsets(flippedTriangleId1, {PeriodicOffset{}, PeriodicOffset{}, PeriodicOffset{}});
        }

        std::array<EdgeKey, 4> outerEdges = {
            MeshQueries2D::makeEdgeKey(edgeNode0, flip->oppositeNode0),
            MeshQueries2D::makeEdgeKey(edgeNode0, flip->oppositeNode1),
            MeshQueries2D::makeEdgeKey(edgeNode1, flip->oppositeNode0),
            MeshQueries2D::makeEdgeKey(edgeNode1, flip->oppositeNode1)};

        for (const auto& edgeKey : outerEdges)
        {
            if (!constrainedEdgeKeys.contains(edgeKey))
                edgeStack.push_back(edgeKey);
        }
    }
}

std::optional<NodeId> MeshOperations2D::splitConstrainedSegment(
    const CurveSegment& segment,
    const Geometry2D::IEdge2D& parentEdge)
{
    auto node1 = meshData_.getNode(segment.nodeId1);
    auto node2 = meshData_.getNode(segment.nodeId2);

    if (node1 == nullptr || node2 == nullptr)
    {
        spdlog::error("splitConstrainedSegment: Invalid node IDs {} or {}", segment.nodeId1.value(), segment.nodeId2.value());
        return std::nullopt;
    }

    double midParameter = (segment.tStart + segment.tEnd) * 0.5;
    Point2D midPoint = parentEdge.getPoint(midParameter);

    // A node can already sit at the midpoint, e.g. a seam twin whose UV midpoint
    // coincides with a node just inserted for the original segment.
    auto existingNodeId = findCoincidentNode(meshData_, midPoint);

    // On a straight constraint (e.g. a seam edge) the midpoint is the chord midpoint and
    // splitTrianglesAtEdge partitions correctly. On a curved one (a circular arc) it is off
    // the chord, and splitting would overlap because the midpoint lies outside one of the
    // two adjacent triangles.
    bool midOnChord = isOnChord(node1->getCoordinates(), node2->getCoordinates(), midPoint);

    if (!existingNodeId && !midOnChord)
    {
        // Bowyer-Watson rebuilds the local triangulation from a star-shaped cavity, which
        // handles a midpoint off the chord.
        NodeId newNodeId = insertVertexBowyerWatson(midPoint, {segment.edgeId});
        enforceEdge(segment.nodeId1, newNodeId);
        enforceEdge(newNodeId, segment.nodeId2);

        mutator_->splitCurveSegment(segment.nodeId1, segment.nodeId2, newNodeId, midParameter);
        return newNodeId;
    }

    // Bowyer-Watson would be wrong here: the midpoint lies on the constraint, so the
    // cavity boundary always contains the old constraint edge, and the triangle it would
    // form with the segment's end nodes is collinear and gets skipped, leaving a mesh hole.
    // Splitting the two adjacent triangles at the midpoint always forms valid triangles.
    NodeId newNodeId = existingNodeId ? *existingNodeId : mutator_->addBoundaryNode(midPoint, {segment.edgeId});

    // The sub-segments must be constrained before the Lawson flip, or it flips them away.
    mutator_->splitCurveSegment(segment.nodeId1, segment.nodeId2, newNodeId, midParameter);

    if (midOnChord)
    {
        auto newTriangleIds = splitTrianglesAtEdge(segment.nodeId1, segment.nodeId2, newNodeId);
        if (!newTriangleIds.empty())
            lawsonFlip(newTriangleIds);
    }
    else
    {
        // An existing node coincides with the arc midpoint but lies off the chord, where
        // splitTrianglesAtEdge would produce clockwise sub-triangles. enforceEdge rebuilds
        // the local triangulation by ear clipping instead.
        enforceEdge(segment.nodeId1, newNodeId);
        enforceEdge(newNodeId, segment.nodeId2);
    }

    return newNodeId;
}

std::vector<ElementId> MeshOperations2D::removeExteriorTriangles(const std::unordered_set<ElementId>& interiorTriangles)
{
    std::vector<ElementId> trianglesToRemove;
    for (const auto& [elementId, element] : meshData_.getElements())
    {
        if (!interiorTriangles.contains(elementId))
        {
            trianglesToRemove.push_back(elementId);
        }
    }

    spdlog::debug("removeExteriorTriangles: Removing {} triangles (outside or in holes)",
                  trianglesToRemove.size());

    for (ElementId elementId : trianglesToRemove)
    {
        mutator_->removeElement(elementId);
    }

    spdlog::debug("removeExteriorTriangles: Complete - {} triangles remaining",
                  meshData_.getElements().size());

    return trianglesToRemove;
}

std::vector<ElementId> MeshOperations2D::classifyAndRemoveExteriorTriangles()
{
    auto interiorTriangles = queries_.classifyTrianglesInteriorExterior();
    return removeExteriorTriangles(interiorTriangles);
}

namespace
{

EdgeOffsetMap collectBoundaryEdgeOffsets(const MeshData2D& meshData,
                                         const PeriodicOffsetTable2D& offsetTable,
                                         const std::vector<ElementId>& triangleIds)
{
    EdgeOffsetMap edgeOffsets;
    for (ElementId triangleId : triangleIds)
    {
        if (!offsetTable.hasOffsets(triangleId))
            continue;

        auto triangle = dynamic_cast<const TriangleElement*>(meshData.getElement(triangleId));
        if (!triangle)
            continue;

        auto& triangleOffsets = offsetTable.getOffsets(triangleId);
        auto& nodeIds = triangle->getNodeIdArray();

        for (int corner = 0; corner < 3; ++corner)
        {
            int index0 = corner;
            int index1 = (corner + 1) % 3;
            if (nodeIds[index0] > nodeIds[index1])
                std::swap(index0, index1);
            edgeOffsets[{nodeIds[index0], nodeIds[index1]}] = {triangleOffsets[index0], triangleOffsets[index1]};
        }
    }
    return edgeOffsets;
}

// The offsets of the edge's two nodes in the edge's own order; zero when the edge has none.
std::array<PeriodicOffset, 2> findEdgeOffsets(const EdgeOffsetMap& edgeOffsets, const std::array<NodeId, 2>& edge)
{
    auto it = edgeOffsets.find(MeshQueries2D::makeEdgeKey(edge[0], edge[1]));
    if (it == edgeOffsets.end())
        return {};
    if (edge[0] <= edge[1])
        return {it->second[0], it->second[1]};
    return {it->second[1], it->second[0]};
}

// Walks the closed cavity boundary from startNodeId, always to the first unvisited neighbour.
std::vector<NodeId> traceBoundaryLoop(const std::vector<std::array<NodeId, 2>>& boundary, NodeId startNodeId)
{
    std::unordered_map<NodeId, std::vector<NodeId>> adjacency;
    for (const auto& edge : boundary)
    {
        adjacency[edge[0]].push_back(edge[1]);
        adjacency[edge[1]].push_back(edge[0]);
    }

    std::vector<NodeId> boundaryLoop{startNodeId};
    std::unordered_set<NodeId> visited{startNodeId};

    NodeId current = startNodeId;
    while (true)
    {
        auto& neighbors = adjacency[current];
        auto next = std::find_if(neighbors.begin(), neighbors.end(),
                                 [&](NodeId neighbor)
                                 { return !visited.contains(neighbor); });
        if (next == neighbors.end())
            break;

        current = *next;
        boundaryLoop.push_back(current);
        visited.insert(current);
    }
    return boundaryLoop;
}

void triangulatePolygon(const MeshData2D& meshData, MeshMutator2D& mutator, std::vector<NodeId> polygon)
{
    orientCounterClockwise(meshData, polygon);

    while (polygon.size() > 3)
    {
        if (!clipEar(meshData, mutator, polygon))
        {
            spdlog::warn("enforceEdge: ear clipping failed for polygon with {} vertices", polygon.size());
            break;
        }
    }

    if (polygon.size() == 3)
        addFinalTriangle(meshData, mutator, polygon);
}

void orientCounterClockwise(const MeshData2D& meshData, std::vector<NodeId>& polygon)
{
    double totalArea = 0.0;
    for (size_t i = 0; i < polygon.size(); ++i)
    {
        auto& point0 = meshData.getNode(polygon[i])->getCoordinates();
        auto& point1 = meshData.getNode(polygon[(i + 1) % polygon.size()])->getCoordinates();
        totalArea += (point0.x() * point1.y() - point1.x() * point0.y());
    }
    if (totalArea < 0.0)
    {
        std::reverse(polygon.begin(), polygon.end());
    }
}

// Cuts off the first ear of the polygon as a triangle; false when the polygon has none.
bool clipEar(const MeshData2D& meshData, MeshMutator2D& mutator, std::vector<NodeId>& polygon)
{
    for (size_t i = 0; i < polygon.size(); ++i)
    {
        size_t previous = (i + polygon.size() - 1) % polygon.size();
        size_t next = (i + 1) % polygon.size();
        if (!isEar(meshData, polygon, previous, i, next))
            continue;

        mutator.addElement(std::make_unique<TriangleElement>(
            std::array<NodeId, 3>{polygon[previous], polygon[i], polygon[next]}));
        polygon.erase(polygon.begin() + i);
        return true;
    }
    return false;
}

// An ear is a convex, non-degenerate corner whose triangle contains no other polygon vertex.
bool isEar(const MeshData2D& meshData, const std::vector<NodeId>& polygon, size_t previous, size_t current, size_t next)
{
    auto& previousPoint = meshData.getNode(polygon[previous])->getCoordinates();
    auto& currentPoint = meshData.getNode(polygon[current])->getCoordinates();
    auto& nextPoint = meshData.getNode(polygon[next])->getCoordinates();

    double area = GeometryUtilities2D::computeSignedArea(previousPoint, currentPoint, nextPoint);
    if (area <= MIN_TRIANGLE_AREA)
        return false;

    for (size_t j = 0; j < polygon.size(); ++j)
    {
        if (j == previous || j == current || j == next)
            continue;
        auto& testPoint = meshData.getNode(polygon[j])->getCoordinates();
        if (GeometryUtilities2D::isPointInsideOrOnTriangle(testPoint, previousPoint, currentPoint, nextPoint))
            return false;
    }
    return true;
}

void addFinalTriangle(const MeshData2D& meshData, MeshMutator2D& mutator, const std::vector<NodeId>& polygon)
{
    auto& point0 = meshData.getNode(polygon[0])->getCoordinates();
    auto& point1 = meshData.getNode(polygon[1])->getCoordinates();
    auto& point2 = meshData.getNode(polygon[2])->getCoordinates();
    double area = GeometryUtilities2D::computeSignedArea(point0, point1, point2);

    if (std::abs(area) < MIN_TRIANGLE_AREA)
        return;

    if (area > 0)
    {
        mutator.addElement(std::make_unique<TriangleElement>(
            std::array<NodeId, 3>{polygon[0], polygon[1], polygon[2]}));
    }
    else
    {
        mutator.addElement(std::make_unique<TriangleElement>(
            std::array<NodeId, 3>{polygon[0], polygon[2], polygon[1]}));
    }
}

EdgeKeySet collectConstrainedEdgeKeys(const MeshData2D& meshData)
{
    EdgeKeySet constrainedEdgeKeys;
    for (const auto& [id, segment] : meshData.getCurveSegmentManager().getAllSegments())
    {
        constrainedEdgeKeys.insert(MeshQueries2D::makeEdgeKey(segment.nodeId1, segment.nodeId2));
    }
    return constrainedEdgeKeys;
}

std::vector<EdgeKey> collectUnconstrainedEdges(const MeshData2D& meshData,
                                               const std::vector<ElementId>& triangleIds,
                                               const EdgeKeySet& constrainedEdgeKeys)
{
    std::vector<EdgeKey> edges;
    for (ElementId triangleId : triangleIds)
    {
        auto triangle = dynamic_cast<const TriangleElement*>(meshData.getElement(triangleId));
        if (!triangle)
            continue;

        for (size_t i = 0; i < 3; ++i)
        {
            auto edge = triangle->getEdge(i);
            auto edgeKey = MeshQueries2D::makeEdgeKey(edge[0], edge[1]);
            if (!constrainedEdgeKeys.contains(edgeKey))
                edges.push_back(edgeKey);
        }
    }
    return edges;
}

// The flip of the edge shared by the two adjacent triangles, when the quad they form is
// convex, the edge is not locally Delaunay, and neither replacement triangle is degenerate.
std::optional<EdgeFlip> findDelaunayFlip(const MeshData2D& meshData,
                                         const ElementGeometry2D& elementGeometry,
                                         NodeId edgeNode0,
                                         NodeId edgeNode1,
                                         const std::vector<ElementId>& adjacent)
{
    auto triangle0 = dynamic_cast<const TriangleElement*>(meshData.getElement(adjacent[0]));
    auto triangle1 = dynamic_cast<const TriangleElement*>(meshData.getElement(adjacent[1]));
    if (!triangle0 || !triangle1)
        return std::nullopt;

    NodeId oppositeNode0 = triangle0->getOppositeNode(edgeNode0, edgeNode1);
    NodeId oppositeNode1 = triangle1->getOppositeNode(edgeNode0, edgeNode1);

    auto edgeNodeObject0 = meshData.getNode(edgeNode0);
    auto edgeNodeObject1 = meshData.getNode(edgeNode1);
    auto oppositeNodeObject0 = meshData.getNode(oppositeNode0);
    auto oppositeNodeObject1 = meshData.getNode(oppositeNode1);
    if (!edgeNodeObject0 || !edgeNodeObject1 || !oppositeNodeObject0 || !oppositeNodeObject1)
        return std::nullopt;

    auto& edgePoint0 = edgeNodeObject0->getCoordinates();
    auto& edgePoint1 = edgeNodeObject1->getCoordinates();
    auto& oppositePoint0 = oppositeNodeObject0->getCoordinates();
    auto& oppositePoint1 = oppositeNodeObject1->getCoordinates();

    // Flipping a non-convex quad produces invalid triangles
    double orientation0 = GeometryUtilities2D::computeOrientation(edgePoint0, edgePoint1, oppositePoint0);
    double orientation1 = GeometryUtilities2D::computeOrientation(edgePoint0, edgePoint1, oppositePoint1);
    if (orientation0 * orientation1 >= 0.0)
        return std::nullopt;

    auto circle = elementGeometry.computeCircumcircle(*triangle0);
    if (!circle.has_value())
        return std::nullopt;

    if (!GeometryUtilities2D::isPointInsideCircle(*circle, oppositePoint1))
        return std::nullopt;

    // A flip can produce collinear vertices, e.g. when the edge being flipped is the
    // seam-split edge and both opposite vertices lie on the same seam boundary line.
    double flipArea0 = GeometryUtilities2D::computeSignedArea(edgePoint0, oppositePoint0, oppositePoint1);
    double flipArea1 = GeometryUtilities2D::computeSignedArea(edgePoint1, oppositePoint1, oppositePoint0);
    if (std::abs(flipArea0) < MIN_FLIP_AREA || std::abs(flipArea1) < MIN_FLIP_AREA)
        return std::nullopt;

    // The edge key is ordered by node id (min, max), not by winding, so the orientation of
    // the replacement triangles comes from their signed areas.
    EdgeFlip flip{oppositeNode0,
                  oppositeNode1,
                  {edgeNode0, oppositeNode0, oppositeNode1},
                  {edgeNode1, oppositeNode1, oppositeNode0}};
    if (flipArea0 < 0)
        std::swap(flip.triangle0[1], flip.triangle0[2]);
    if (flipArea1 < 0)
        std::swap(flip.triangle1[1], flip.triangle1[2]);
    return flip;
}

std::optional<NodeId> findCoincidentNode(const MeshData2D& meshData, const Point2D& point)
{
    for (const auto& [nodeId, node] : meshData.getNodes())
    {
        if ((node->getCoordinates() - point).squaredNorm() < NODE_COINCIDENCE_TOLERANCE * NODE_COINCIDENCE_TOLERANCE)
            return nodeId;
    }
    return std::nullopt;
}

// True when the point lies on the line through the chord; a degenerate chord counts as on.
bool isOnChord(const Point2D& chordStart, const Point2D& chordEnd, const Point2D& point)
{
    Point2D chord = chordEnd - chordStart;
    double chordLengthSquared = chord.squaredNorm();
    if (chordLengthSquared > MIN_CHORD_LENGTH_SQUARED)
    {
        Point2D projected = chordStart + chord.dot(point - chordStart) / chordLengthSquared * chord;
        return (point - projected).squaredNorm() < MAX_CHORD_DEVIATION_SQUARED;
    }
    return true;
}

} // namespace

} // namespace Meshing
