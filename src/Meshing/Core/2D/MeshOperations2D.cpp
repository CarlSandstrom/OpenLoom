#include "MeshOperations2D.h"
#include "Common/Exceptions/MeshException.h"
#include "Geometry/2D/Base/IEdge2D.h"
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

    // Retriangulate the cavity by tracing the boundary into two sub-polygons
    // (one on each side of the constraint edge) and ear-clipping each.

    // Build adjacency map: for each vertex, which other vertices are connected by boundary edges
    std::unordered_map<size_t, std::vector<size_t>> adjacency;
    for (const auto& edge : boundary)
    {
        adjacency[edge[0]].push_back(edge[1]);
        adjacency[edge[1]].push_back(edge[0]);
    }

    // Trace the boundary polygon starting from nodeId1.
    // The boundary forms a closed polygon containing nodeId1 and nodeId2.
    // We trace from nodeId1 in one direction until we hit nodeId2 (left sub-polygon),
    // then continue until we return to nodeId1 (right sub-polygon).
    std::vector<size_t> fullBoundary;
    fullBoundary.push_back(nodeId1);

    std::unordered_set<size_t> visited;
    visited.insert(nodeId1);

    size_t current = nodeId1;
    while (true)
    {
        bool advanced = false;
        for (size_t neighbor : adjacency[current])
        {
            if (visited.count(neighbor) == 0)
            {
                fullBoundary.push_back(neighbor);
                visited.insert(neighbor);
                current = neighbor;
                advanced = true;
                break;
            }
        }
        if (!advanced)
            break;
    }

    // Split the full boundary into left and right sub-polygons at nodeId1 and nodeId2
    // Find nodeId2's position in the boundary
    auto it = std::find(fullBoundary.begin(), fullBoundary.end(), nodeId2);
    if (it == fullBoundary.end())
    {
        spdlog::warn("enforceEdge: nodeId2 ({}) not found in traced boundary for edge ({}, {})",
                     nodeId2, nodeId1, nodeId2);
        return false;
    }
    size_t splitIdx = std::distance(fullBoundary.begin(), it);

    // Left sub-polygon: nodeId1 → ... → nodeId2
    std::vector<size_t> leftPolygon(fullBoundary.begin(), fullBoundary.begin() + splitIdx + 1);
    // Right sub-polygon: nodeId2 → ... → nodeId1
    std::vector<size_t> rightPolygon(fullBoundary.begin() + splitIdx, fullBoundary.end());
    rightPolygon.push_back(nodeId1);

    // Triangulate each sub-polygon using ear clipping
    const double MIN_TRIANGLE_AREA = 1e-10;

    auto triangulatePolygon = [&](std::vector<size_t> polygon)
    {
        // Ensure CCW winding
        double totalArea = 0.0;
        for (size_t i = 0; i < polygon.size(); ++i)
        {
            const Point2D& pi = meshData_.getNode(polygon[i])->getCoordinates();
            const Point2D& pj = meshData_.getNode(polygon[(i + 1) % polygon.size()])->getCoordinates();
            totalArea += (pi.x() * pj.y() - pj.x() * pi.y());
        }
        if (totalArea < 0.0)
        {
            std::reverse(polygon.begin(), polygon.end());
        }

        // Ear clipping
        while (polygon.size() > 3)
        {
            bool earFound = false;
            for (size_t i = 0; i < polygon.size(); ++i)
            {
                size_t prev = (i + polygon.size() - 1) % polygon.size();
                size_t next = (i + 1) % polygon.size();

                const Point2D& pPrev = meshData_.getNode(polygon[prev])->getCoordinates();
                const Point2D& pCurr = meshData_.getNode(polygon[i])->getCoordinates();
                const Point2D& pNext = meshData_.getNode(polygon[next])->getCoordinates();

                double area = GeometryUtilities2D::computeSignedArea(pPrev, pCurr, pNext);
                if (area <= MIN_TRIANGLE_AREA)
                    continue; // Not a convex vertex (or degenerate)

                // Check that no other polygon vertex is inside this ear
                bool isEar = true;
                for (size_t j = 0; j < polygon.size(); ++j)
                {
                    if (j == prev || j == i || j == next)
                        continue;
                    const Point2D& pTest = meshData_.getNode(polygon[j])->getCoordinates();
                    // Point-in-triangle test using orientation
                    double d1 = GeometryUtilities2D::computeOrientation(pPrev, pCurr, pTest);
                    double d2 = GeometryUtilities2D::computeOrientation(pCurr, pNext, pTest);
                    double d3 = GeometryUtilities2D::computeOrientation(pNext, pPrev, pTest);
                    bool hasNeg = (d1 < 0) || (d2 < 0) || (d3 < 0);
                    bool hasPos = (d1 > 0) || (d2 > 0) || (d3 > 0);
                    if (!(hasNeg && hasPos))
                    {
                        isEar = false;
                        break;
                    }
                }

                if (isEar)
                {
                    mutator_->addElement(std::make_unique<TriangleElement>(
                        std::array<size_t, 3>{polygon[prev], polygon[i], polygon[next]}));
                    polygon.erase(polygon.begin() + i);
                    earFound = true;
                    break;
                }
            }
            if (!earFound)
            {
                spdlog::warn("enforceEdge: ear clipping failed for polygon with {} vertices", polygon.size());
                break;
            }
        }

        // Final triangle
        if (polygon.size() == 3)
        {
            const Point2D& p0 = meshData_.getNode(polygon[0])->getCoordinates();
            const Point2D& p1 = meshData_.getNode(polygon[1])->getCoordinates();
            const Point2D& p2 = meshData_.getNode(polygon[2])->getCoordinates();
            double area = GeometryUtilities2D::computeSignedArea(p0, p1, p2);

            if (std::abs(area) >= MIN_TRIANGLE_AREA)
            {
                if (area > 0)
                {
                    mutator_->addElement(std::make_unique<TriangleElement>(
                        std::array<size_t, 3>{polygon[0], polygon[1], polygon[2]}));
                }
                else
                {
                    mutator_->addElement(std::make_unique<TriangleElement>(
                        std::array<size_t, 3>{polygon[0], polygon[2], polygon[1]}));
                }
            }
        }
    };

    triangulatePolygon(leftPolygon);
    triangulatePolygon(rightPolygon);

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

        // Get coordinates of the quad vertices
        const Node2D* nodeA = meshData_.getNode(a);
        const Node2D* nodeB = meshData_.getNode(b);
        const Node2D* nodeC = meshData_.getNode(c);
        const Node2D* nodeD = meshData_.getNode(d);
        if (!nodeA || !nodeB || !nodeC || !nodeD)
            continue;

        const Point2D& pA = nodeA->getCoordinates();
        const Point2D& pB = nodeB->getCoordinates();
        const Point2D& pC = nodeC->getCoordinates();
        const Point2D& pD = nodeD->getCoordinates();

        // Skip if quad is not convex — flipping a non-convex quad produces invalid triangles
        double orientC = GeometryUtilities2D::computeOrientation(pA, pB, pC);
        double orientD = GeometryUtilities2D::computeOrientation(pA, pB, pD);
        if (orientC * orientD >= 0.0)
            continue;

        // Check Delaunay criterion: is d inside circumcircle of (a, b, c)?
        auto circle = geom.computeCircumcircle(*elem1);
        if (!circle.has_value())
            continue;

        if (!GeometryUtilities2D::isPointInsideCircle(*circle, pD))
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

    // Insert midpoint via Bowyer-Watson and re-enforce constraint edges
    size_t newNodeId = insertVertexBowyerWatson(midPoint, {tMid}, {edgeId});

    enforceEdge(segment.nodeId1, newNodeId);
    enforceEdge(newNodeId, segment.nodeId2);

    // Update constrained segments: replace old with two new
    ConstrainedSegment2D seg1{segment.nodeId1, newNodeId, segment.role};
    ConstrainedSegment2D seg2{newNodeId, segment.nodeId2, segment.role};
    mutator_->replaceConstrainedSegment(segment, seg1, seg2);

    // Remove artifact triangle caused by concave constrained edges.
    // After splitting (A, B) at M, edge (A, B) should no longer exist in the mesh.
    // If exactly one triangle has edge (A, B), it's an artifact of the curved edge
    // approximation. If two triangles share it, it's an internal boundary — keep both.
    auto artifactTriangles = queries_.findTrianglesAdjacentToEdge(segment.nodeId1, segment.nodeId2);
    if (artifactTriangles.size() == 1)
    {
        spdlog::debug("splitConstrainedSegment: Removing artifact triangle {} (contains old edge ({}, {}))",
                      artifactTriangles[0], segment.nodeId1, segment.nodeId2);
        mutator_->removeElement(artifactTriangles[0]);
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
