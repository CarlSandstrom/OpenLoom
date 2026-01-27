#include "MeshOperations2D.h"
#include "Common/Exceptions/MeshException.h"
#include "ConstraintChecker2D.h"
#include "ElementGeometry2D.h"
#include "Geometry/2D/Base/IEdge2D.h"
#include "GeometryUtilities2D.h"
#include "Meshing/Data/2D/MeshMutator2D.h"
#include "Meshing/Data/2D/Node2D.h"
#include "Topology2D/Edge2D.h"
#include "Topology2D/Topology2D.h"
#include "spdlog/spdlog.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace Meshing
{

MeshOperations2D::MeshOperations2D(MeshData2D& meshData) :
    meshData_(meshData),
    mutator_(std::make_unique<MeshMutator2D>(meshData)),
    geometry_(std::make_unique<ElementGeometry2D>(meshData))
{
}

size_t MeshOperations2D::insertVertexBowyerWatson(const Point2D& point,
                                                  const std::vector<double>& edgeParameters,
                                                  const std::vector<std::string>& edgeIds)
{
    std::vector<size_t> conflicting = findConflictingTriangles(point);

    if (conflicting.empty())
    {
        SPDLOG_WARN("MeshOperations2D: No conflicting triangles found for point ({}, {})",
                    point.x(), point.y());
        CMESH_THROW_CODE(cMesh::MeshException,
                         cMesh::MeshException::ErrorCode::INVALID_OPERATION,
                         "No conflicting triangles found for point (" + std::to_string(point.x()) + ", " + std::to_string(point.y()) + ")");
    }

    std::vector<std::array<size_t, 2>> boundary = findCavityBoundary(conflicting);

    for (auto conflictingElementId : conflicting)
    {
        mutator_->removeElement(conflictingElementId);
    }

    size_t newVertex;
    if (!edgeParameters.empty() && !edgeIds.empty())
    {
        newVertex = mutator_->addBoundaryNode(point, edgeParameters, edgeIds);
    }
    else
    {
        newVertex = mutator_->addNode(point);
    }

    // Helper to compute signed area for degenerate triangle detection
    auto computeSignedArea = [&](size_t n1, size_t n2, size_t n3) -> double
    {
        const Point2D& p1 = meshData_.getNode(n1)->getCoordinates();
        const Point2D& p2 = meshData_.getNode(n2)->getCoordinates();
        const Point2D& p3 = meshData_.getNode(n3)->getCoordinates();
        return 0.5 * ((p2.x() - p1.x()) * (p3.y() - p1.y()) -
                      (p3.x() - p1.x()) * (p2.y() - p1.y()));
    };

    const double MIN_TRIANGLE_AREA = 1e-10;

    for (const auto& edge : boundary)
    {
        // Check if triangle would be degenerate before creating it
        double area = computeSignedArea(newVertex, edge[0], edge[1]);

        if (std::abs(area) < MIN_TRIANGLE_AREA)
        {
            spdlog::debug("Skipping degenerate triangle in Bowyer-Watson: ({}, {}, {})",
                          newVertex, edge[0], edge[1]);
            continue;
        }

        auto newTriangle = std::make_unique<TriangleElement>(std::array<size_t, 3>{newVertex, edge[0], edge[1]});
        mutator_->addElement(std::move(newTriangle));
    }

    return newVertex;
}

std::vector<size_t> MeshOperations2D::findConflictingTriangles(const Point2D& point) const
{
    std::vector<size_t> conflicting;

    for (const auto& [id, element] : meshData_.getElements())
    {
        const auto* triangle = dynamic_cast<const TriangleElement*>(element.get());
        if (triangle == nullptr)
        {
            SPDLOG_WARN("MeshOperations2D: Skipping non-triangle element {}", id);
            continue;
        }

        auto circle = geometry_->computeCircumcircle(*triangle);

        if (circle && GeometryUtilities2D::isPointInsideCircle(*circle, point))
        {
            conflicting.push_back(id);
        }
    }

    return conflicting;
}

std::vector<std::array<size_t, 2>> MeshOperations2D::findCavityBoundary(const std::vector<size_t>& conflictingIndices) const
{
    // Count how many times each edge appears
    std::map<std::pair<size_t, size_t>, int> edgeCount;
    std::map<std::pair<size_t, size_t>, std::array<size_t, 2>> edgeLookup;

    for (size_t idx : conflictingIndices)
    {
        auto tri = dynamic_cast<const TriangleElement&>(*meshData_.getElement(idx));
        for (size_t i = 0; i < 3; ++i)
        {
            auto edge = tri.getEdge(i);
            auto key = makeEdgeKey(edge[0], edge[1]);
            edgeCount[key]++;
            edgeLookup[key] = edge; // Store original order
        }
    }

    // Boundary edges appear exactly once
    std::vector<std::array<size_t, 2>> boundary;
    for (const auto& [key, count] : edgeCount)
    {
        if (count == 1)
        {
            boundary.push_back(edgeLookup.at(key));
        }
    }

    return boundary;
}

bool MeshOperations2D::removeTrianglesContainingNode(size_t nodeId)
{
    std::vector<size_t> elementsToRemove;
    for (const auto& [id, element] : meshData_.getElements())
    {
        const auto* triangle = dynamic_cast<const TriangleElement*>(element.get());
        if (triangle && triangle->getHasNode(nodeId))
        {
            elementsToRemove.push_back(id);
        }
    }

    for (size_t id : elementsToRemove)
    {
        mutator_->removeElement(id);
    }

    return !elementsToRemove.empty();
}

std::vector<size_t> MeshOperations2D::findIntersectingTriangles(size_t nodeId1, size_t nodeId2) const
{
    std::vector<size_t> result;

    const auto& p1 = meshData_.getNode(nodeId1)->getCoordinates();
    const auto& p2 = meshData_.getNode(nodeId2)->getCoordinates();

    for (auto& idAndTriangle : meshData_.getElements())
    {
        auto triangle = dynamic_cast<const TriangleElement*>(idAndTriangle.second.get());
        if (!triangle)
        {
            spdlog::warn("MeshOperations2D: Skipping non-triangle element {}", idAndTriangle.first);
            continue; // Skip non-triangle elements
        }

        // Skip triangles that already contain both nodes
        if (triangle->getHasNode(nodeId1) && triangle->getHasNode(nodeId2))
        {
            continue;
        }

        // Check if constraint edge intersects any edge of this triangle
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

            const Point2D& coordinate1 = meshData_.getNode(edge[0])->getCoordinates();
            const Point2D& coordinate2 = meshData_.getNode(edge[1])->getCoordinates();

            if (segmentsIntersect(p1, p2, coordinate1, coordinate2))
            {
                intersects = true;
                break;
            }
        }

        if (intersects)
        {
            result.push_back(idAndTriangle.first);
        }
    }

    return result;
}

bool MeshOperations2D::enforceEdge(size_t nodeId1, size_t nodeId2)
{
    // First check if the edge already exists in the mesh
    for (const auto& [id, element] : meshData_.getElements())
    {
        const auto* triangle = dynamic_cast<const TriangleElement*>(element.get());
        if (!triangle)
            continue;

        // Check all three edges of the triangle
        for (size_t i = 0; i < 3; ++i)
        {
            auto edge = triangle->getEdge(i);
            if ((edge[0] == nodeId1 && edge[1] == nodeId2) ||
                (edge[0] == nodeId2 && edge[1] == nodeId1))
            {
                spdlog::debug("Edge ({}, {}) already exists in mesh", nodeId1, nodeId2);
                return true; // Edge already exists
            }
        }
    }

    auto intersectingTriangles = findIntersectingTriangles(nodeId1, nodeId2);

    if (intersectingTriangles.empty())
    {
        spdlog::warn("Edge ({}, {}) does not exist but no intersecting triangles found", nodeId1, nodeId2);
        return true; // No triangles to remove, edge might already be enforced
    }

    // Find the cavity boundary
    std::vector<std::array<size_t, 2>> boundary = findCavityBoundary(intersectingTriangles);

    // Remove intersecting triangles
    for (auto index : intersectingTriangles)
    {
        const auto& tri = *meshData_.getElement(index);
        mutator_->removeElement(index);
    }

    // Retriangulate the cavity
    // The cavity boundary needs to be split into two polygons by the constraint edge

    // Separate boundary edges into left and right of the constraint edge
    std::vector<size_t> leftPolygon;
    std::vector<size_t> rightPolygon;

    const Point2D& p1 = meshData_.getNode(nodeId1)->getCoordinates();
    const Point2D& p2 = meshData_.getNode(nodeId2)->getCoordinates();

    // Collect all unique vertices on the boundary
    std::vector<size_t> boundaryVertices;
    for (const auto& edge : boundary)
    {
        // Only add vertices that aren't the constraint edge endpoints
        for (size_t v : edge)
        {
            if (v != nodeId1 && v != nodeId2)
            {
                if (std::find(boundaryVertices.begin(), boundaryVertices.end(), v) == boundaryVertices.end())
                {
                    boundaryVertices.push_back(v);
                }
            }
        }
    }

    // Determine which side of the constraint edge each vertex is on
    auto orientation = [](const Point2D& p, const Point2D& q, const Point2D& r) -> double
    {
        return (q.x() - p.x()) * (r.y() - p.y()) - (q.y() - p.y()) * (r.x() - p.x());
    };

    for (size_t v : boundaryVertices)
    {
        const Point2D& point = meshData_.getNode(v)->getCoordinates();
        double orient = orientation(p1, p2, point);

        if (orient > 1e-10)
        {
            leftPolygon.push_back(v);
        }
        else if (orient < -1e-10)
        {
            rightPolygon.push_back(v);
        }
    }

    // Sort vertices along the constraint edge to create proper polygon ordering
    auto sortVerticesAlongEdge = [&](std::vector<size_t>& vertices)
    {
        if (vertices.empty()) return;

        // Project each vertex onto the constraint edge to get ordering parameter
        Point2D edgeDir = p2 - p1;
        double edgeLength2 = edgeDir.x() * edgeDir.x() + edgeDir.y() * edgeDir.y();

        std::sort(vertices.begin(), vertices.end(), [&](size_t v1, size_t v2)
                  {
            const Point2D& pv1 = meshData_.getNode(v1)->getCoordinates();
            const Point2D& pv2 = meshData_.getNode(v2)->getCoordinates();

            // Project onto edge direction
            Point2D p1ToV1 = pv1 - p1;
            Point2D p1ToV2 = pv2 - p1;
            double t1 = (p1ToV1.x() * edgeDir.x() + p1ToV1.y() * edgeDir.y()) / edgeLength2;
            double t2 = (p1ToV2.x() * edgeDir.x() + p1ToV2.y() * edgeDir.y()) / edgeLength2;

            return t1 < t2; });
    };

    sortVerticesAlongEdge(leftPolygon);
    sortVerticesAlongEdge(rightPolygon);

    // Helper function to compute signed area for orientation checking
    auto computeSignedArea = [&](size_t n1, size_t n2, size_t n3) -> double
    {
        const Point2D& p1 = meshData_.getNode(n1)->getCoordinates();
        const Point2D& p2 = meshData_.getNode(n2)->getCoordinates();
        const Point2D& p3 = meshData_.getNode(n3)->getCoordinates();
        return 0.5 * ((p2.x() - p1.x()) * (p3.y() - p1.y()) -
                      (p3.x() - p1.x()) * (p2.y() - p1.y()));
    };

    // Triangulate left polygon using fan triangulation from nodeId1
    // Creates triangles: (nodeId1, leftPolygon[i], leftPolygon[i+1])
    // and final edge triangle (nodeId1, leftPolygon.back(), nodeId2)
    const double MIN_TRIANGLE_AREA = 1e-10;

    if (!leftPolygon.empty())
    {
        for (size_t i = 0; i < leftPolygon.size() - 1; ++i)
        {
            // Check orientation and adjust if needed
            double area = computeSignedArea(nodeId1, leftPolygon[i], leftPolygon[i + 1]);

            // Skip degenerate triangles (collinear or nearly collinear points)
            if (std::abs(area) < MIN_TRIANGLE_AREA)
            {
                spdlog::debug("Skipping degenerate triangle in left polygon: ({}, {}, {})",
                              nodeId1, leftPolygon[i], leftPolygon[i + 1]);
                continue;
            }

            if (area > 0)
            {
                auto newTriangle = std::make_unique<TriangleElement>(
                    std::array<size_t, 3>{nodeId1, leftPolygon[i], leftPolygon[i + 1]});
                mutator_->addElement(std::move(newTriangle));
            }
            else
            {
                auto newTriangle = std::make_unique<TriangleElement>(
                    std::array<size_t, 3>{nodeId1, leftPolygon[i + 1], leftPolygon[i]});
                mutator_->addElement(std::move(newTriangle));
            }
        }
        // Final triangle connects to nodeId2
        double area = computeSignedArea(nodeId1, leftPolygon.back(), nodeId2);

        // Skip degenerate triangles
        if (std::abs(area) >= MIN_TRIANGLE_AREA)
        {
            if (area > 0)
            {
                auto newTriangle = std::make_unique<TriangleElement>(
                    std::array<size_t, 3>{nodeId1, leftPolygon.back(), nodeId2});
                mutator_->addElement(std::move(newTriangle));
            }
            else
            {
                auto newTriangle = std::make_unique<TriangleElement>(
                    std::array<size_t, 3>{nodeId1, nodeId2, leftPolygon.back()});
                mutator_->addElement(std::move(newTriangle));
            }
        }
        else
        {
            spdlog::debug("Skipping degenerate final triangle in left polygon: ({}, {}, {})",
                          nodeId1, leftPolygon.back(), nodeId2);
        }
    }
    else
    {
        // No vertices on left side, just need one triangle for the constraint edge
        // This case is already handled by the right polygon or is degenerate
    }

    // Triangulate right polygon using fan triangulation from nodeId1
    // Creates triangles with correct orientation
    if (!rightPolygon.empty())
    {
        for (size_t i = 0; i < rightPolygon.size() - 1; ++i)
        {
            // Check orientation and adjust if needed
            double area = computeSignedArea(nodeId1, rightPolygon[i + 1], rightPolygon[i]);

            // Skip degenerate triangles (collinear or nearly collinear points)
            if (std::abs(area) < MIN_TRIANGLE_AREA)
            {
                spdlog::debug("Skipping degenerate triangle in right polygon: ({}, {}, {})",
                              nodeId1, rightPolygon[i + 1], rightPolygon[i]);
                continue;
            }

            if (area > 0)
            {
                auto newTriangle = std::make_unique<TriangleElement>(
                    std::array<size_t, 3>{nodeId1, rightPolygon[i + 1], rightPolygon[i]});
                mutator_->addElement(std::move(newTriangle));
            }
            else
            {
                auto newTriangle = std::make_unique<TriangleElement>(
                    std::array<size_t, 3>{nodeId1, rightPolygon[i], rightPolygon[i + 1]});
                mutator_->addElement(std::move(newTriangle));
            }
        }
        // Final triangle connects to nodeId2
        double area = computeSignedArea(nodeId1, nodeId2, rightPolygon.back());

        // Skip degenerate triangles
        if (std::abs(area) >= MIN_TRIANGLE_AREA)
        {
            if (area > 0)
            {
                auto newTriangle = std::make_unique<TriangleElement>(
                    std::array<size_t, 3>{nodeId1, nodeId2, rightPolygon.back()});
                mutator_->addElement(std::move(newTriangle));
            }
            else
            {
                auto newTriangle = std::make_unique<TriangleElement>(
                    std::array<size_t, 3>{nodeId1, rightPolygon.back(), nodeId2});
                mutator_->addElement(std::move(newTriangle));
            }
        }
        else
        {
            spdlog::debug("Skipping degenerate final triangle in right polygon: ({}, {}, {})",
                          nodeId1, nodeId2, rightPolygon.back());
        }
    }
    else
    {
        // No vertices on right side
    }

    return true;
}

void MeshOperations2D::retriangulate(size_t vertexNodeId,
                                     const std::vector<std::array<size_t, 2>>& boundary,
                                     std::vector<TriangleElement>& activeTriangles) const
{
    for (const auto& edge : boundary)
    {
        activeTriangles.emplace_back(std::array<size_t, 3>{vertexNodeId, edge[0], edge[1]});
    }
}

bool MeshOperations2D::segmentsIntersect(const Point2D& a1, const Point2D& a2,
                                         const Point2D& b1, const Point2D& b2) const
{
    // Compute orientation of ordered triplet (p, q, r)
    // Returns: 0 -> colinear, 1 -> clockwise, 2 -> counterclockwise
    auto orientation = [](const Point2D& p, const Point2D& q, const Point2D& r) -> int
    {
        double val = (q.y() - p.y()) * (r.x() - q.x()) -
                     (q.x() - p.x()) * (r.y() - q.y());

        if (std::abs(val) < 1e-10) return 0;
        return (val > 0) ? 1 : 2;
    };

    // Check if point q lies on segment pr
    auto onSegment = [](const Point2D& p, const Point2D& q, const Point2D& r) -> bool
    {
        return q.x() <= std::max(p.x(), r.x()) && q.x() >= std::min(p.x(), r.x()) &&
               q.y() <= std::max(p.y(), r.y()) && q.y() >= std::min(p.y(), r.y());
    };

    int o1 = orientation(a1, a2, b1);
    int o2 = orientation(a1, a2, b2);
    int o3 = orientation(b1, b2, a1);
    int o4 = orientation(b1, b2, a2);

    // General case
    if (o1 != o2 && o3 != o4)
        return true;

    // Special cases for collinear points
    if (o1 == 0 && onSegment(a1, b1, a2)) return true;
    if (o2 == 0 && onSegment(a1, b2, a2)) return true;
    if (o3 == 0 && onSegment(b1, a1, b2)) return true;
    if (o4 == 0 && onSegment(b1, a2, b2)) return true;

    return false;
}

std::vector<ConstrainedSegment2D> MeshOperations2D::extractConstrainedEdges(
    const Topology2D::Topology2D& topology,
    const std::map<std::string, size_t>& cornerIdToPointIndexMap,
    const std::map<size_t, size_t>& pointIndexToNodeIdMap,
    const std::map<std::string, std::vector<size_t>>& edgeIdToPointIndicesMap) const
{
    std::vector<ConstrainedSegment2D> constrainedEdges;

    for (const auto& edgeId : topology.getAllEdgeIds())
    {
        auto edgePointsIt = edgeIdToPointIndicesMap.find(edgeId);

        if (edgePointsIt != edgeIdToPointIndicesMap.end() && edgePointsIt->second.size() >= 2)
        {
            const auto& pointIndices = edgePointsIt->second;

            for (size_t i = 0; i < pointIndices.size() - 1; ++i)
            {
                size_t startPointIdx = pointIndices[i];
                size_t endPointIdx = pointIndices[i + 1];

                size_t startNodeId = pointIndexToNodeIdMap.at(startPointIdx);
                size_t endNodeId = pointIndexToNodeIdMap.at(endPointIdx);

                constrainedEdges.push_back(ConstrainedSegment2D{startNodeId, endNodeId});

                spdlog::info("Edge {} segment {}: Node IDs ({}, {})", edgeId, i, startNodeId, endNodeId);
            }
        }
        else
        {
            const auto edgeTopology = topology.getEdge(edgeId);

            size_t startNodeId = pointIndexToNodeIdMap.at(
                cornerIdToPointIndexMap.at(edgeTopology.getStartCornerId()));
            size_t endNodeId = pointIndexToNodeIdMap.at(
                cornerIdToPointIndexMap.at(edgeTopology.getEndCornerId()));

            constrainedEdges.push_back(ConstrainedSegment2D{startNodeId, endNodeId});

            spdlog::info("Edge {}: Node IDs ({}, {})", edgeId, startNodeId, endNodeId);
        }
    }

    return constrainedEdges;
}

std::optional<std::pair<ConstrainedSegment2D, ConstrainedSegment2D>> MeshOperations2D::splitConstrainedSegment(
    const ConstrainedSegment2D& segment,
    const Geometry2D::IEdge2D& parentEdge)
{
    const Node2D* node1 = meshData_.getNode(segment.nodeId1);
    const Node2D* node2 = meshData_.getNode(segment.nodeId2);

    if (node1 == nullptr || node2 == nullptr)
    {
        spdlog::error("splitConstrainedSegment: Invalid node IDs {} or {}", segment.nodeId1, segment.nodeId2);
        return std::nullopt;
    }

    const auto& t1Params = node1->getEdgeParameters();
    const auto& t2Params = node2->getEdgeParameters();
    const auto& geometryIds1 = node1->getGeometryIds();
    const auto& geometryIds2 = node2->getGeometryIds();

    if (t1Params.empty() || t2Params.empty())
    {
        spdlog::error("splitConstrainedSegment: Nodes {} and {} must have edge parameters", segment.nodeId1, segment.nodeId2);
        return std::nullopt;
    }

    // Find the edge parameter index that corresponds to the parent edge
    std::string edgeId = parentEdge.getId();
    auto it1 = std::find(geometryIds1.begin(), geometryIds1.end(), edgeId);
    auto it2 = std::find(geometryIds2.begin(), geometryIds2.end(), edgeId);

    if (it1 == geometryIds1.end() || it2 == geometryIds2.end())
    {
        spdlog::error("splitConstrainedSegment: Parent edge {} not found in node geometry IDs", edgeId);
        return std::nullopt;
    }

    size_t idx1 = std::distance(geometryIds1.begin(), it1);
    size_t idx2 = std::distance(geometryIds2.begin(), it2);

    double t1 = t1Params[idx1];
    double t2 = t2Params[idx2];
    double tMid = (t1 + t2) * 0.5;
    Point2D midPoint = parentEdge.getPoint(tMid);

    size_t newNodeId = insertVertexBowyerWatson(midPoint, {tMid}, {edgeId});

    enforceEdge(segment.nodeId1, newNodeId);
    enforceEdge(newNodeId, segment.nodeId2);

    ConstrainedSegment2D seg1{segment.nodeId1, newNodeId};
    ConstrainedSegment2D seg2{newNodeId, segment.nodeId2};

    return std::make_pair(seg1, seg2);
}

// Flood fill-based triangle classification
void MeshOperations2D::classifyTrianglesInteriorExterior(const std::vector<ConstrainedSegment2D>& constrainedEdges)
{
    if (meshData_.getElements().empty())
    {
        spdlog::warn("classifyTrianglesInteriorExterior: No triangles in mesh");
        return;
    }

    if (constrainedEdges.empty())
    {
        spdlog::warn("classifyTrianglesInteriorExterior: No constraint edges, skipping classification");
        return;
    }

    // Helper: Check if an edge is a constraint edge
    auto isConstraintEdge = [&](size_t nodeId1, size_t nodeId2) -> bool
    {
        for (const auto& segment : constrainedEdges)
        {
            if ((segment.nodeId1 == nodeId1 && segment.nodeId2 == nodeId2) ||
                (segment.nodeId1 == nodeId2 && segment.nodeId2 == nodeId1))
            {
                return true;
            }
        }
        return false;
    };

    // Helper: Compute distance from point to line segment
    auto pointToSegmentDistance = [](const Point2D& p, const Point2D& a, const Point2D& b) -> double
    {
        Point2D ab = b - a;
        Point2D ap = p - a;

        double abLengthSq = ab.x() * ab.x() + ab.y() * ab.y();
        if (abLengthSq < 1e-12)
        {
            return std::sqrt(ap.x() * ap.x() + ap.y() * ap.y());
        }

        double t = (ap.x() * ab.x() + ap.y() * ab.y()) / abLengthSq;
        t = std::max(0.0, std::min(1.0, t));

        Point2D projection = a + Point2D(ab.x() * t, ab.y() * t);
        Point2D diff = p - projection;
        return std::sqrt(diff.x() * diff.x() + diff.y() * diff.y());
    };

    // Helper: Compute centroid of triangle
    auto computeCentroid = [&](const TriangleElement* triangle) -> Point2D
    {
        const auto& nodeIds = triangle->getNodeIdArray();
        Point2D p0 = meshData_.getNode(nodeIds[0])->getCoordinates();
        Point2D p1 = meshData_.getNode(nodeIds[1])->getCoordinates();
        Point2D p2 = meshData_.getNode(nodeIds[2])->getCoordinates();
        return Point2D((p0.x() + p1.x() + p2.x()) / 3.0,
                       (p0.y() + p1.y() + p2.y()) / 3.0);
    };

    // Step 1: Find seed triangle (farthest from all constraint edges)
    TriangleElement* seedTriangle = nullptr;
    size_t seedTriangleId = SIZE_MAX;
    double maxMinDistance = -1.0;

    for (const auto& [elemId, element] : meshData_.getElements())
    {
        auto* triangle = dynamic_cast<TriangleElement*>(element.get());
        if (!triangle)
            continue;

        Point2D centroid = computeCentroid(triangle);
        double minDistToConstraint = std::numeric_limits<double>::max();

        for (const auto& segment : constrainedEdges)
        {
            const Node2D* node1 = meshData_.getNode(segment.nodeId1);
            const Node2D* node2 = meshData_.getNode(segment.nodeId2);

            if (!node1 || !node2)
                continue;

            Point2D p1 = node1->getCoordinates();
            Point2D p2 = node2->getCoordinates();
            double dist = pointToSegmentDistance(centroid, p1, p2);
            minDistToConstraint = std::min(minDistToConstraint, dist);
        }

        if (minDistToConstraint > maxMinDistance)
        {
            maxMinDistance = minDistToConstraint;
            seedTriangle = triangle;
            seedTriangleId = elemId;
        }
    }

    if (!seedTriangle || seedTriangleId == SIZE_MAX)
    {
        spdlog::error("classifyTrianglesInteriorExterior: Could not find seed triangle");
        return;
    }

    spdlog::info("classifyTrianglesInteriorExterior: Found seed triangle {} with min distance {} to constraints",
                 seedTriangleId, maxMinDistance);

    // Step 2: Build edge-to-triangles adjacency map
    using EdgeKey = std::pair<size_t, size_t>;
    struct EdgeKeyHash
    {
        std::size_t operator()(const EdgeKey& key) const
        {
            return std::hash<size_t>{}(key.first) ^ (std::hash<size_t>{}(key.second) << 1);
        }
    };
    std::unordered_map<EdgeKey, std::vector<size_t>, EdgeKeyHash> edgeToTriangles;

    for (const auto& [elemId, element] : meshData_.getElements())
    {
        const auto* triangle = dynamic_cast<const TriangleElement*>(element.get());
        if (!triangle)
            continue;

        for (size_t i = 0; i < 3; ++i)
        {
            auto edge = triangle->getEdge(i);
            EdgeKey key = makeEdgeKey(edge[0], edge[1]);
            edgeToTriangles[key].push_back(elemId);
        }
    }

    // Step 3: Perform BFS flood fill starting from seed triangle
    std::unordered_set<size_t> insideTriangles;
    std::queue<size_t> queue;

    queue.push(seedTriangleId);
    insideTriangles.insert(seedTriangleId);

    size_t visitedCount = 0;
    while (!queue.empty())
    {
        size_t currentTriId = queue.front();
        queue.pop();
        visitedCount++;

        const IElement* elem = meshData_.getElement(currentTriId);
        if (!elem)
            continue;

        const auto* triangle = dynamic_cast<const TriangleElement*>(elem);
        if (!triangle)
            continue;

        // Check all three edges of the current triangle
        for (size_t i = 0; i < 3; ++i)
        {
            auto edge = triangle->getEdge(i);
            size_t node1 = edge[0];
            size_t node2 = edge[1];

            // Don't cross constraint edges
            if (isConstraintEdge(node1, node2))
            {
                continue;
            }

            // Find adjacent triangle through this edge
            EdgeKey edgeKey = makeEdgeKey(node1, node2);
            auto it = edgeToTriangles.find(edgeKey);
            if (it == edgeToTriangles.end())
            {
                continue;
            }

            const auto& adjacentTriangles = it->second;

            // Find the neighbor (the triangle that is not currentTriId)
            for (size_t neighborId : adjacentTriangles)
            {
                if (neighborId == currentTriId)
                    continue;

                // If we haven't visited this neighbor yet, add it to the flood fill
                if (insideTriangles.find(neighborId) == insideTriangles.end())
                {
                    insideTriangles.insert(neighborId);
                    queue.push(neighborId);
                }
            }
        }
    }

    spdlog::info("classifyTrianglesInteriorExterior: Flood fill visited {} triangles", visitedCount);
    spdlog::info("classifyTrianglesInteriorExterior: Marked {} triangles as inside", insideTriangles.size());

    // Step 4: Remove triangles that were not reached (outside or in holes)
    std::vector<size_t> trianglesToRemove;
    for (const auto& [elemId, element] : meshData_.getElements())
    {
        if (insideTriangles.find(elemId) == insideTriangles.end())
        {
            trianglesToRemove.push_back(elemId);
        }
    }

    spdlog::info("classifyTrianglesInteriorExterior: Removing {} triangles (outside or in holes)",
                 trianglesToRemove.size());

    for (size_t elemId : trianglesToRemove)
    {
        mutator_->removeElement(elemId);
    }

    spdlog::info("classifyTrianglesInteriorExterior: Complete - {} triangles remaining",
                 meshData_.getElements().size());
}

std::vector<ConstrainedSegment2D> MeshOperations2D::findEncroachedSegments(
    const std::vector<ConstrainedSegment2D>& constrainedSegments) const
{
    std::vector<ConstrainedSegment2D> encroached;

    ConstraintChecker2D checker(meshData_);

    for (const auto& segment : constrainedSegments)
    {
        for (const auto& [nodeId, node] : meshData_.getNodes())
        {
            if (nodeId == segment.nodeId1 || nodeId == segment.nodeId2)
                continue;

            Point2D point = node->getCoordinates();

            if (checker.isSegmentEncroached(segment, point))
            {
                encroached.push_back(segment);
                break;
            }
        }
    }

    return encroached;
}

std::vector<ConstrainedSegment2D> MeshOperations2D::findSegmentsEncroachedByPoint(
    const Point2D& point,
    const std::vector<ConstrainedSegment2D>& constrainedSegments) const
{
    std::vector<ConstrainedSegment2D> encroached;

    ConstraintChecker2D checker(meshData_);

    for (const auto& segment : constrainedSegments)
    {
        if (checker.isSegmentEncroached(segment, point))
        {
            encroached.push_back(segment);
        }
    }

    return encroached;
}

} // namespace Meshing
