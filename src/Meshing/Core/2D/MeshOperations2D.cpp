#include "MeshOperations2D.h"
#include "Common/Exceptions/MeshException.h"
#include "Geometry/2D/Base/GeometryOperations2D.h"
#include "Geometry/2D/Base/IEdge2D.h"
#include "Geometry/2D/Base/IFace2D.h"
#include "GeometryUtilities2D.h"
#include "Meshing/Data/2D/MeshMutator2D.h"
#include "Meshing/Data/2D/Node2D.h"
#include "spdlog/spdlog.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace Meshing
{

MeshOperations2D::MeshOperations2D(MeshData2D& meshData) :
    meshData_(meshData),
    queries_(meshData),
    mutator_(std::make_unique<MeshMutator2D>(meshData)),
    geometry_(std::make_unique<ElementGeometry2D>(meshData))
{
}

size_t MeshOperations2D::insertVertexBowyerWatson(const Point2D& point,
                                                  const std::vector<double>& edgeParameters,
                                                  const std::vector<std::string>& edgeIds)
{
    std::vector<size_t> conflicting = queries_.findConflictingTriangles(point);

    if (conflicting.empty())
    {
        SPDLOG_WARN("MeshOperations2D: No conflicting triangles found for point ({}, {})",
                    point.x(), point.y());
        CMESH_THROW_CODE(cMesh::MeshException,
                         cMesh::MeshException::ErrorCode::INVALID_OPERATION,
                         "No conflicting triangles found for point (" + std::to_string(point.x()) + ", " + std::to_string(point.y()) + ")");
    }

    std::vector<std::array<size_t, 2>> boundary = queries_.findCavityBoundary(conflicting);

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

    const double MIN_TRIANGLE_AREA = 1e-10;

    for (const auto& edge : boundary)
    {
        // Check if triangle would be degenerate before creating it
        const Point2D& pNew = meshData_.getNode(newVertex)->getCoordinates();
        const Point2D& pE0 = meshData_.getNode(edge[0])->getCoordinates();
        const Point2D& pE1 = meshData_.getNode(edge[1])->getCoordinates();
        double area = GeometryUtilities2D::computeSignedArea(pNew, pE0, pE1);

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

    auto intersectingTriangles = queries_.findIntersectingTriangles(nodeId1, nodeId2);

    if (intersectingTriangles.empty())
    {
        spdlog::warn("Edge ({}, {}) does not exist but no intersecting triangles found", nodeId1, nodeId2);
        return true; // No triangles to remove, edge might already be enforced
    }

    // Find the cavity boundary
    std::vector<std::array<size_t, 2>> boundary = queries_.findCavityBoundary(intersectingTriangles);

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
    for (size_t v : boundaryVertices)
    {
        const Point2D& point = meshData_.getNode(v)->getCoordinates();
        double orient = GeometryUtilities2D::computeOrientation(p1, p2, point);

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
        const Point2D& pt1 = meshData_.getNode(n1)->getCoordinates();
        const Point2D& pt2 = meshData_.getNode(n2)->getCoordinates();
        const Point2D& pt3 = meshData_.getNode(n3)->getCoordinates();
        return GeometryUtilities2D::computeSignedArea(pt1, pt2, pt3);
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

std::vector<size_t> MeshOperations2D::splitTrianglesAtEdge(size_t edgeNode1, size_t edgeNode2, size_t midNodeId)
{
    auto adjacentTriangles = queries_.findTrianglesAdjacentToEdge(edgeNode1, edgeNode2);
    std::vector<size_t> newTriangleIds;

    for (size_t triId : adjacentTriangles)
    {
        const auto* element = meshData_.getElement(triId);
        const auto* triangle = dynamic_cast<const TriangleElement*>(element);
        if (!triangle)
            continue;

        // Find the opposite vertex (the one not on the split edge)
        const auto& nodes = triangle->getNodeIdArray();
        size_t opposite = 0;
        for (size_t n : nodes)
        {
            if (n != edgeNode1 && n != edgeNode2)
            {
                opposite = n;
                break;
            }
        }

        mutator_->removeElement(triId);

        size_t t1 = mutator_->addElement(
            std::make_unique<TriangleElement>(std::array<size_t, 3>{edgeNode1, midNodeId, opposite}));
        size_t t2 = mutator_->addElement(
            std::make_unique<TriangleElement>(std::array<size_t, 3>{midNodeId, edgeNode2, opposite}));

        newTriangleIds.push_back(t1);
        newTriangleIds.push_back(t2);
    }

    return newTriangleIds;
}

void MeshOperations2D::lawsonFlip(const std::vector<size_t>& newTriangleIds)
{
    // Build set of constrained edge keys for fast lookup
    using EdgeKey = std::pair<size_t, size_t>;
    struct EdgeKeyHash
    {
        std::size_t operator()(const EdgeKey& key) const
        {
            return std::hash<size_t>{}(key.first) ^ (std::hash<size_t>{}(key.second) << 1);
        }
    };

    std::unordered_set<EdgeKey, EdgeKeyHash> constrainedEdgeKeys;
    for (const auto& seg : meshData_.getConstrainedSegments())
    {
        constrainedEdgeKeys.insert(MeshQueries2D::makeEdgeKey(seg.nodeId1, seg.nodeId2));
    }

    auto isConstrained = [&](size_t a, size_t b) -> bool
    {
        return constrainedEdgeKeys.count(MeshQueries2D::makeEdgeKey(a, b)) > 0;
    };

    // Collect initial edges to check from new triangles
    std::vector<EdgeKey> edgeStack;

    for (size_t triId : newTriangleIds)
    {
        const auto* element = meshData_.getElement(triId);
        const auto* triangle = dynamic_cast<const TriangleElement*>(element);
        if (!triangle)
            continue;

        for (size_t i = 0; i < 3; ++i)
        {
            auto edge = triangle->getEdge(i);
            if (!isConstrained(edge[0], edge[1]))
            {
                edgeStack.push_back(MeshQueries2D::makeEdgeKey(edge[0], edge[1]));
            }
        }
    }

    ElementGeometry2D geom(meshData_);

    while (!edgeStack.empty())
    {
        auto [a, b] = edgeStack.back();
        edgeStack.pop_back();

        // Find the two triangles sharing this edge
        auto adjacent = queries_.findTrianglesAdjacentToEdge(a, b);
        if (adjacent.size() != 2)
            continue;

        const auto* elem1 = dynamic_cast<const TriangleElement*>(meshData_.getElement(adjacent[0]));
        const auto* elem2 = dynamic_cast<const TriangleElement*>(meshData_.getElement(adjacent[1]));
        if (!elem1 || !elem2)
            continue;

        // Find opposite vertices
        size_t c = 0, d = 0;
        for (size_t n : elem1->getNodeIdArray())
        {
            if (n != a && n != b)
            {
                c = n;
                break;
            }
        }
        for (size_t n : elem2->getNodeIdArray())
        {
            if (n != a && n != b)
            {
                d = n;
                break;
            }
        }

        // Check Delaunay criterion: is d inside circumcircle of (a, b, c)?
        auto circle = geom.computeCircumcircle(*elem1);
        if (!circle.has_value())
            continue;

        const Node2D* nodeD = meshData_.getNode(d);
        if (!nodeD)
            continue;

        if (!GeometryUtilities2D::isPointInsideCircle(*circle, nodeD->getCoordinates()))
            continue;

        // Flip: remove (a,b,c) and (a,b,d), add (a,c,d) and (b,d,c)
        mutator_->removeElement(adjacent[0]);
        mutator_->removeElement(adjacent[1]);

        mutator_->addElement(
            std::make_unique<TriangleElement>(std::array<size_t, 3>{a, c, d}));
        mutator_->addElement(
            std::make_unique<TriangleElement>(std::array<size_t, 3>{b, d, c}));

        // Push the 4 outer edges of the flipped quad for further checking
        std::array<EdgeKey, 4> outerEdges = {
            MeshQueries2D::makeEdgeKey(a, c),
            MeshQueries2D::makeEdgeKey(a, d),
            MeshQueries2D::makeEdgeKey(b, c),
            MeshQueries2D::makeEdgeKey(b, d)};

        for (const auto& edgeKey : outerEdges)
        {
            if (!isConstrained(edgeKey.first, edgeKey.second))
            {
                edgeStack.push_back(edgeKey);
            }
        }
    }
}

std::optional<size_t> MeshOperations2D::splitConstrainedSegment(
    const ConstrainedSegment2D& segment,
    const Geometry2D::IEdge2D& parentEdge,
    const Geometry2D::IFace2D& domainFace)
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

    // Insert midpoint via Bowyer-Watson and re-enforce constraint edges
    size_t newNodeId = insertVertexBowyerWatson(midPoint, {tMid}, {edgeId});

    enforceEdge(segment.nodeId1, newNodeId);
    enforceEdge(newNodeId, segment.nodeId2);

    // Update constrained segments: replace old with two new
    ConstrainedSegment2D seg1{segment.nodeId1, newNodeId, segment.role};
    ConstrainedSegment2D seg2{newNodeId, segment.nodeId2, segment.role};
    mutator_->replaceConstrainedSegment(segment, seg1, seg2);

    // Remove artifact triangles caused by concave constrained edges.
    // If the midpoint of the old straight-line edge (A, B) is outside the domain,
    // any triangle still having edge (A, B) is an artifact crossing a hole.
    Point2D oldEdgeMidpoint = (node1->getCoordinates() + node2->getCoordinates()) * 0.5;
    if (!Geometry2D::GeometryOperations2D::isPointInsideDomain(oldEdgeMidpoint, domainFace))
    {
        auto artifactTriangles = queries_.findTrianglesAdjacentToEdge(segment.nodeId1, segment.nodeId2);
        for (size_t triId : artifactTriangles)
        {
            spdlog::debug("splitConstrainedSegment: Removing artifact triangle {} (old edge midpoint outside domain)",
                          triId);
            mutator_->removeElement(triId);
        }
    }

    return newNodeId;
}

std::vector<size_t> MeshOperations2D::removeExteriorTriangles(const std::unordered_set<size_t>& interiorTriangles)
{
    std::vector<size_t> trianglesToRemove;
    for (const auto& [elemId, element] : meshData_.getElements())
    {
        if (interiorTriangles.find(elemId) == interiorTriangles.end())
        {
            trianglesToRemove.push_back(elemId);
        }
    }

    spdlog::info("removeExteriorTriangles: Removing {} triangles (outside or in holes)",
                 trianglesToRemove.size());

    for (size_t elemId : trianglesToRemove)
    {
        mutator_->removeElement(elemId);
    }

    spdlog::info("removeExteriorTriangles: Complete - {} triangles remaining",
                 meshData_.getElements().size());

    return trianglesToRemove;
}

} // namespace Meshing
