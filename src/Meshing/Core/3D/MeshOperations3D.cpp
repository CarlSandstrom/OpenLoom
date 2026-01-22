#include "MeshOperations3D.h"
#include "ConstraintChecker3D.h"
#include "ElementGeometry3D.h"
#include "ElementQuality3D.h"
#include "Geometry/3D/Base/IEdge3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "GeometryUtilities3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Meshing/Data/3D/Node3D.h"
#include "Meshing/Data/Base/MeshConnectivity.h"
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

MeshOperations3D::MeshOperations3D(MeshData3D& meshData) :
    meshData_(meshData),
    mutator_(std::make_unique<MeshMutator3D>(meshData))
{
}

std::array<size_t, 4> MeshOperations3D::createBoundingTetrahedron(const std::vector<Point3D>& points)
{
    if (points.empty())
    {
        spdlog::error("MeshOperations3D::createBoundingTetrahedron: Empty point list");
        // Return dummy IDs - caller should check for empty input
        return {0, 0, 0, 0};
    }

    // Compute bounding box of all points
    double minX = std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double minZ = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double maxY = std::numeric_limits<double>::lowest();
    double maxZ = std::numeric_limits<double>::lowest();

    for (const auto& p : points)
    {
        minX = std::min(minX, p.x());
        minY = std::min(minY, p.y());
        minZ = std::min(minZ, p.z());
        maxX = std::max(maxX, p.x());
        maxY = std::max(maxY, p.y());
        maxZ = std::max(maxZ, p.z());
    }

    // Add margin to ensure all points are strictly inside
    double dx = maxX - minX;
    double dy = maxY - minY;
    double dz = maxZ - minZ;
    double maxDim = std::max({dx, dy, dz, 1.0}); // At least 1.0 to handle degenerate cases
    double margin = maxDim * 10.0; // Large margin for numerical stability

    // Create a super-tetrahedron that contains the bounding box
    // Using a tetrahedron with one vertex at origin-margin and three vertices
    // extending far along each axis
    Point3D v0(minX - margin, minY - margin, minZ - margin);
    Point3D v1(maxX + 3.0 * margin, minY - margin, minZ - margin);
    Point3D v2(minX - margin, maxY + 3.0 * margin, minZ - margin);
    Point3D v3(minX - margin, minY - margin, maxZ + 3.0 * margin);

    // Add the four bounding vertices
    size_t id0 = mutator_->addNode(v0);
    size_t id1 = mutator_->addNode(v1);
    size_t id2 = mutator_->addNode(v2);
    size_t id3 = mutator_->addNode(v3);

    // Create the bounding tetrahedron
    auto boundingTet = std::make_unique<TetrahedralElement>(
        std::array<size_t, 4>{id0, id1, id2, id3});
    mutator_->addElement(std::move(boundingTet));

    spdlog::debug("MeshOperations3D::createBoundingTetrahedron: Created with nodes ({}, {}, {}, {})",
                  id0, id1, id2, id3);

    return {id0, id1, id2, id3};
}

std::vector<size_t> MeshOperations3D::initializeDelaunay(const std::vector<Point3D>& points)
{
    if (points.empty())
    {
        spdlog::warn("MeshOperations3D::initializeDelaunay: Empty point list");
        return {};
    }

    spdlog::info("MeshOperations3D::initializeDelaunay: Initializing with {} points", points.size());

    // Create the bounding tetrahedron
    std::array<size_t, 4> boundingIds = createBoundingTetrahedron(points);

    // Insert each point using Bowyer-Watson
    std::vector<size_t> nodeIds;
    nodeIds.reserve(points.size());

    for (size_t i = 0; i < points.size(); ++i)
    {
        size_t nodeId = insertVertexBowyerWatson(points[i]);
        nodeIds.push_back(nodeId);

        if ((i + 1) % 100 == 0)
        {
            spdlog::debug("MeshOperations3D::initializeDelaunay: Inserted {}/{} points",
                          i + 1, points.size());
        }
    }

    spdlog::info("MeshOperations3D::initializeDelaunay: Inserted {} points, {} tetrahedra",
                 nodeIds.size(), meshData_.getElementCount());

    return nodeIds;
}

void MeshOperations3D::removeBoundingTetrahedron(const std::array<size_t, 4>& boundingNodeIds)
{
    spdlog::debug("MeshOperations3D::removeBoundingTetrahedron: Removing bounding nodes ({}, {}, {}, {})",
                  boundingNodeIds[0], boundingNodeIds[1], boundingNodeIds[2], boundingNodeIds[3]);

    // Create a set of bounding node IDs for fast lookup
    std::unordered_set<size_t> boundingSet(boundingNodeIds.begin(), boundingNodeIds.end());

    // Find all tetrahedra that contain any bounding vertex
    std::vector<size_t> tetsToRemove;
    for (const auto& [tetId, element] : meshData_.getElements())
    {
        const auto* tet = dynamic_cast<const TetrahedralElement*>(element.get());
        if (!tet)
        {
            continue;
        }

        const auto& nodes = tet->getNodeIds();
        for (size_t nodeId : nodes)
        {
            if (boundingSet.count(nodeId) > 0)
            {
                tetsToRemove.push_back(tetId);
                break;
            }
        }
    }

    // Remove the tetrahedra
    for (size_t tetId : tetsToRemove)
    {
        mutator_->removeElement(tetId);
    }

    // Remove the bounding nodes
    for (size_t nodeId : boundingNodeIds)
    {
        mutator_->removeNode(nodeId);
    }

    spdlog::info("MeshOperations3D::removeBoundingTetrahedron: Removed {} tetrahedra and 4 bounding nodes",
                 tetsToRemove.size());
}

size_t MeshOperations3D::insertVertexBowyerWatson(const Point3D& point,
                                                  const std::vector<double>& edgeParameters,
                                                  const std::vector<std::string>& edgeIds)
{
    // Find conflicting tetrahedra (those whose circumsphere contains the point)
    std::vector<size_t> conflicting = findConflictingTetrahedra(point);

    if (conflicting.empty())
    {
        spdlog::warn("MeshOperations3D::insertVertexBowyerWatson: No conflicting tetrahedra found");
        // Just add the vertex without removing any tetrahedra
        size_t nodeId = mutator_->addNode(point);
        return nodeId;
    }

    // Find the cavity boundary
    std::vector<std::array<size_t, 3>> boundary = findCavityBoundary(conflicting);

    // Remove conflicting tetrahedra
    for (size_t tetId : conflicting)
    {
        mutator_->removeElement(tetId);
    }

    // Insert the new vertex (use boundary node if geometry IDs provided)
    size_t nodeId;
    if (!edgeParameters.empty() || !edgeIds.empty())
    {
        nodeId = mutator_->addBoundaryNode(point, edgeParameters, edgeIds);
    }
    else
    {
        nodeId = mutator_->addNode(point);
    }

    // Retriangulate the cavity with the new vertex
    retriangulate(nodeId, boundary);

    return nodeId;
}

std::vector<size_t> MeshOperations3D::findConflictingTetrahedra(const Point3D& point) const
{
    std::vector<size_t> conflicting;
    ElementGeometry3D geometry(meshData_);

    // Check all tetrahedra
    for (const auto& [tetId, element] : meshData_.getElements())
    {
        const auto* tet = dynamic_cast<const TetrahedralElement*>(element.get());
        if (!tet)
        {
            continue;
        }

        // Check if point is inside the circumsphere
        if (geometry.isPointInsideCircumscribingSphere(*tet, point))
        {
            conflicting.push_back(tetId);
        }
    }

    return conflicting;
}

std::vector<std::array<size_t, 3>>
MeshOperations3D::findCavityBoundary(const std::vector<size_t>& conflictingIndices) const
{
    // Build a set of conflicting tetrahedra for fast lookup
    std::unordered_set<size_t> conflictingSet(conflictingIndices.begin(), conflictingIndices.end());

    // Map to count how many times each face appears
    std::map<std::array<size_t, 3>, size_t> faceCount;

    // Iterate through all conflicting tetrahedra and count their faces
    for (size_t tetId : conflictingIndices)
    {
        const auto* element = meshData_.getElement(tetId);
        const auto* tet = dynamic_cast<const TetrahedralElement*>(element);
        if (!tet)
        {
            continue;
        }

        // Get the four faces of the tetrahedron
        const auto& nodes = tet->getNodeIds();
        std::array<std::array<size_t, 3>, 4> faces = {{{nodes[0], nodes[1], nodes[2]},
                                                       {nodes[0], nodes[1], nodes[3]},
                                                       {nodes[0], nodes[2], nodes[3]},
                                                       {nodes[1], nodes[2], nodes[3]}}};

        for (auto& face : faces)
        {
            // Normalize face ordering
            std::array<size_t, 3> sortedFace = makeTriangleKey(face[0], face[1], face[2]);
            faceCount[sortedFace]++;
        }
    }

    // Boundary faces appear exactly once (not shared with another conflicting tet)
    std::vector<std::array<size_t, 3>> boundary;
    for (const auto& [face, count] : faceCount)
    {
        if (count == 1)
        {
            boundary.push_back(face);
        }
    }

    return boundary;
}

void MeshOperations3D::retriangulate(size_t vertexNodeId,
                                     const std::vector<std::array<size_t, 3>>& boundary)
{
    // Create a new tetrahedron for each boundary face
    for (const auto& face : boundary)
    {
        auto tet = std::make_unique<TetrahedralElement>(
            std::array<size_t, 4>{vertexNodeId, face[0], face[1], face[2]});
        mutator_->addElement(std::move(tet));
    }
}

std::optional<std::pair<ConstrainedSubsegment3D, ConstrainedSubsegment3D>>
MeshOperations3D::splitConstrainedSubsegment(const ConstrainedSubsegment3D& subsegment,
                                             const Geometry3D::IEdge3D& parentEdge)
{
    // Get the two endpoint nodes
    const auto* node1 = meshData_.getNode(subsegment.nodeId1);
    const auto* node2 = meshData_.getNode(subsegment.nodeId2);

    if (!node1 || !node2)
    {
        spdlog::error("MeshOperations3D::splitConstrainedSubsegment: Invalid node IDs");
        return std::nullopt;
    }

    // Compute the midpoint on the geometry (handles curved edges)
    // For now, use simple Euclidean midpoint
    // TODO: Use parametric midpoint from parent edge when available
    Point3D midpoint = (node1->getCoordinates() + node2->getCoordinates()) * 0.5;

    // Try to get parametric midpoint from edge if possible
    try
    {
        auto [tMin, tMax] = parentEdge.getParameterBounds();
        double tMid = (tMin + tMax) * 0.5;
        midpoint = parentEdge.getPoint(tMid);
    }
    catch (...)
    {
        // Fall back to Euclidean midpoint (already computed)
    }

    // Insert the midpoint vertex
    std::vector<std::string> geometryIds = {subsegment.geometryId};
    size_t midNodeId = insertVertexBowyerWatson(midpoint, {}, geometryIds);

    // Create two new subsegments
    ConstrainedSubsegment3D sub1;
    sub1.nodeId1 = subsegment.nodeId1;
    sub1.nodeId2 = midNodeId;
    sub1.geometryId = subsegment.geometryId;

    ConstrainedSubsegment3D sub2;
    sub2.nodeId1 = midNodeId;
    sub2.nodeId2 = subsegment.nodeId2;
    sub2.geometryId = subsegment.geometryId;

    return std::make_pair(sub1, sub2);
}

std::optional<size_t>
MeshOperations3D::splitConstrainedSubfacet(const ConstrainedSubfacet3D& subfacet,
                                           const Geometry3D::ISurface3D& parentSurface)
{
    // Get the three vertices of the subfacet
    const auto* node1 = meshData_.getNode(subfacet.nodeId1);
    const auto* node2 = meshData_.getNode(subfacet.nodeId2);
    const auto* node3 = meshData_.getNode(subfacet.nodeId3);

    if (!node1 || !node2 || !node3)
    {
        spdlog::error("MeshOperations3D::splitConstrainedSubfacet: Invalid node IDs");
        return std::nullopt;
    }

    Point3D p1 = node1->getCoordinates();
    Point3D p2 = node2->getCoordinates();
    Point3D p3 = node3->getCoordinates();

    // Compute the circumcenter in 3D space
    EquatorialSphere sphere = GeometryUtilities3D::createEquatorialSphere(p1, p2, p3);
    Point3D circumcenter = sphere.center;

    // Project the circumcenter back onto the parent surface if needed
    // For now, use the computed circumcenter directly
    // TODO: Project onto parent surface to handle curved surfaces

    // Insert the circumcenter vertex
    std::vector<std::string> geometryIds = {subfacet.geometryId};
    size_t centerNodeId = insertVertexBowyerWatson(circumcenter, {}, geometryIds);

    return centerNodeId;
}

bool MeshOperations3D::removeTetrahedraContainingNode(size_t nodeId)
{
    bool anyRemoved = false;
    std::vector<size_t> tetsToRemove;

    // Find all tetrahedra containing the node
    for (const auto& [tetId, element] : meshData_.getElements())
    {
        const auto* tet = dynamic_cast<const TetrahedralElement*>(element.get());
        if (!tet)
        {
            continue;
        }

        const auto& nodes = tet->getNodeIds();
        if (std::find(nodes.begin(), nodes.end(), nodeId) != nodes.end())
        {
            tetsToRemove.push_back(tetId);
        }
    }

    // Remove the tetrahedra
    for (size_t tetId : tetsToRemove)
    {
        mutator_->removeElement(tetId);
        anyRemoved = true;
    }

    return anyRemoved;
}

void MeshOperations3D::classifyTetrahedraInteriorExterior(
    const std::vector<ConstrainedSubfacet3D>& constrainedSubfacets)
{
    if (meshData_.getElements().empty())
    {
        spdlog::warn("classifyTetrahedraInteriorExterior: No tetrahedra in mesh");
        return;
    }

    if (constrainedSubfacets.empty())
    {
        spdlog::warn("classifyTetrahedraInteriorExterior: No constraint faces, skipping classification");
        return;
    }

    ElementGeometry3D geometry(meshData_);

    // Step 1: Find seed tetrahedron (farthest from all constraint faces)
    const TetrahedralElement* seedTet = nullptr;
    size_t seedTetId = SIZE_MAX;
    double maxMinDistance = -1.0;

    for (const auto& [elemId, element] : meshData_.getElements())
    {
        const auto* tet = dynamic_cast<const TetrahedralElement*>(element.get());
        if (!tet)
            continue;

        Point3D centroid = geometry.computeCentroid(*tet);
        double minDistToConstraint = std::numeric_limits<double>::max();

        for (const auto& subfacet : constrainedSubfacets)
        {
            const Node3D* node1 = meshData_.getNode(subfacet.nodeId1);
            const Node3D* node2 = meshData_.getNode(subfacet.nodeId2);
            const Node3D* node3 = meshData_.getNode(subfacet.nodeId3);

            if (!node1 || !node2 || !node3)
                continue;

            Point3D p1 = node1->getCoordinates();
            Point3D p2 = node2->getCoordinates();
            Point3D p3 = node3->getCoordinates();
            double dist = GeometryUtilities3D::computePointToTriangleCentroidDistance(centroid, p1, p2, p3);
            minDistToConstraint = std::min(minDistToConstraint, dist);
        }

        if (minDistToConstraint > maxMinDistance)
        {
            maxMinDistance = minDistToConstraint;
            seedTet = tet;
            seedTetId = elemId;
        }
    }

    if (!seedTet || seedTetId == SIZE_MAX)
    {
        spdlog::error("classifyTetrahedraInteriorExterior: Could not find seed tetrahedron");
        return;
    }

    spdlog::info("classifyTetrahedraInteriorExterior: Found seed tet {} with min distance {} to constraints",
                 seedTetId, maxMinDistance);

    // Step 2: Build face-to-tetrahedra adjacency map
    using FaceKey = std::array<size_t, 3>;
    struct FaceKeyHash
    {
        std::size_t operator()(const FaceKey& key) const
        {
            return std::hash<size_t>{}(key[0]) ^
                   (std::hash<size_t>{}(key[1]) << 1) ^
                   (std::hash<size_t>{}(key[2]) << 2);
        }
    };
    std::unordered_map<FaceKey, std::vector<size_t>, FaceKeyHash> faceToTets;

    for (const auto& [elemId, element] : meshData_.getElements())
    {
        const auto* tet = dynamic_cast<const TetrahedralElement*>(element.get());
        if (!tet)
            continue;

        const auto& nodes = tet->getNodeIds();
        // Four faces of the tetrahedron
        std::array<std::array<size_t, 3>, 4> faces = {{{nodes[0], nodes[1], nodes[2]},
                                                        {nodes[0], nodes[1], nodes[3]},
                                                        {nodes[0], nodes[2], nodes[3]},
                                                        {nodes[1], nodes[2], nodes[3]}}};

        for (const auto& face : faces)
        {
            FaceKey key = makeTriangleKey(face[0], face[1], face[2]);
            faceToTets[key].push_back(elemId);
        }
    }

    // Step 3: Perform BFS flood fill starting from seed tetrahedron
    std::unordered_set<size_t> insideTetrahedra;
    std::queue<size_t> queue;

    queue.push(seedTetId);
    insideTetrahedra.insert(seedTetId);

    size_t visitedCount = 0;
    while (!queue.empty())
    {
        size_t currentTetId = queue.front();
        queue.pop();
        visitedCount++;

        const IElement* elem = meshData_.getElement(currentTetId);
        if (!elem)
            continue;

        const auto* tet = dynamic_cast<const TetrahedralElement*>(elem);
        if (!tet)
            continue;

        // Check all four faces of the current tetrahedron
        const auto& nodes = tet->getNodeIds();
        std::array<std::array<size_t, 3>, 4> faces = {{{nodes[0], nodes[1], nodes[2]},
                                                        {nodes[0], nodes[1], nodes[3]},
                                                        {nodes[0], nodes[2], nodes[3]},
                                                        {nodes[1], nodes[2], nodes[3]}}};

        for (const auto& face : faces)
        {
            size_t node1 = face[0];
            size_t node2 = face[1];
            size_t node3 = face[2];

            // Don't cross constraint faces
            if (ConstraintChecker3D::isConstraintFace(node1, node2, node3, constrainedSubfacets))
            {
                continue;
            }

            // Find adjacent tetrahedron through this face
            FaceKey faceKey = makeTriangleKey(node1, node2, node3);
            auto it = faceToTets.find(faceKey);
            if (it == faceToTets.end())
            {
                continue;
            }

            const auto& adjacentTets = it->second;

            // Find the neighbor (the tetrahedron that is not currentTetId)
            for (size_t neighborId : adjacentTets)
            {
                if (neighborId == currentTetId)
                    continue;

                // If we haven't visited this neighbor yet, add it to the flood fill
                if (insideTetrahedra.find(neighborId) == insideTetrahedra.end())
                {
                    insideTetrahedra.insert(neighborId);
                    queue.push(neighborId);
                }
            }
        }
    }

    spdlog::info("classifyTetrahedraInteriorExterior: Flood fill visited {} tetrahedra", visitedCount);
    spdlog::info("classifyTetrahedraInteriorExterior: Marked {} tetrahedra as inside", insideTetrahedra.size());

    // Step 4: Remove tetrahedra that were not reached (outside or in holes)
    std::vector<size_t> tetsToRemove;
    for (const auto& [elemId, element] : meshData_.getElements())
    {
        if (insideTetrahedra.find(elemId) == insideTetrahedra.end())
        {
            tetsToRemove.push_back(elemId);
        }
    }

    spdlog::info("classifyTetrahedraInteriorExterior: Removing {} tetrahedra (outside or in holes)",
                 tetsToRemove.size());

    for (size_t elemId : tetsToRemove)
    {
        mutator_->removeElement(elemId);
    }

    spdlog::info("classifyTetrahedraInteriorExterior: Complete - {} tetrahedra remaining",
                 meshData_.getElements().size());
}

bool MeshOperations3D::edgeExistsInMesh(size_t nodeId1, size_t nodeId2) const
{
    for (const auto& [tetId, element] : meshData_.getElements())
    {
        const auto* tet = dynamic_cast<const TetrahedralElement*>(element.get());
        if (!tet)
            continue;

        const auto& nodes = tet->getNodeIds();
        bool hasNode1 = std::find(nodes.begin(), nodes.end(), nodeId1) != nodes.end();
        bool hasNode2 = std::find(nodes.begin(), nodes.end(), nodeId2) != nodes.end();

        if (hasNode1 && hasNode2)
        {
            return true;
        }
    }
    return false;
}

std::vector<size_t> MeshOperations3D::findTetrahedraWithEdge(size_t nodeId1, size_t nodeId2) const
{
    std::vector<size_t> result;

    for (const auto& [tetId, element] : meshData_.getElements())
    {
        const auto* tet = dynamic_cast<const TetrahedralElement*>(element.get());
        if (!tet)
            continue;

        const auto& nodes = tet->getNodeIds();
        bool hasNode1 = std::find(nodes.begin(), nodes.end(), nodeId1) != nodes.end();
        bool hasNode2 = std::find(nodes.begin(), nodes.end(), nodeId2) != nodes.end();

        if (hasNode1 && hasNode2)
        {
            result.push_back(tetId);
        }
    }
    return result;
}

bool MeshOperations3D::faceExistsInMesh(size_t nodeId1, size_t nodeId2, size_t nodeId3) const
{
    for (const auto& [tetId, element] : meshData_.getElements())
    {
        const auto* tet = dynamic_cast<const TetrahedralElement*>(element.get());
        if (!tet)
            continue;

        const auto& nodes = tet->getNodeIds();
        bool hasNode1 = std::find(nodes.begin(), nodes.end(), nodeId1) != nodes.end();
        bool hasNode2 = std::find(nodes.begin(), nodes.end(), nodeId2) != nodes.end();
        bool hasNode3 = std::find(nodes.begin(), nodes.end(), nodeId3) != nodes.end();

        if (hasNode1 && hasNode2 && hasNode3)
        {
            return true;
        }
    }
    return false;
}

std::vector<size_t> MeshOperations3D::findTetrahedraWithFace(size_t nodeId1, size_t nodeId2, size_t nodeId3) const
{
    std::vector<size_t> result;

    for (const auto& [tetId, element] : meshData_.getElements())
    {
        const auto* tet = dynamic_cast<const TetrahedralElement*>(element.get());
        if (!tet)
            continue;

        const auto& nodes = tet->getNodeIds();
        bool hasNode1 = std::find(nodes.begin(), nodes.end(), nodeId1) != nodes.end();
        bool hasNode2 = std::find(nodes.begin(), nodes.end(), nodeId2) != nodes.end();
        bool hasNode3 = std::find(nodes.begin(), nodes.end(), nodeId3) != nodes.end();

        if (hasNode1 && hasNode2 && hasNode3)
        {
            result.push_back(tetId);
        }
    }
    return result;
}

size_t MeshOperations3D::findOppositeVertex(size_t tetId, size_t faceNode1, size_t faceNode2, size_t faceNode3) const
{
    const auto* element = meshData_.getElement(tetId);
    const auto* tet = dynamic_cast<const TetrahedralElement*>(element);
    if (!tet)
    {
        return SIZE_MAX;
    }

    const auto& nodes = tet->getNodeIds();
    for (size_t nodeId : nodes)
    {
        if (nodeId != faceNode1 && nodeId != faceNode2 && nodeId != faceNode3)
        {
            return nodeId;
        }
    }
    return SIZE_MAX;
}

std::vector<size_t> MeshOperations3D::findSkinnyTetrahedra(double ratioBound) const
{
    std::vector<size_t> skinnyTets;
    ElementGeometry3D geometry(meshData_);
    ElementQuality3D quality(meshData_);

    for (const auto& [tetId, element] : meshData_.getElements())
    {
        const auto* tet = dynamic_cast<const TetrahedralElement*>(element.get());
        if (!tet)
            continue;

        double ratio = quality.getCircumradiusToShortestEdgeRatio(*tet);
        if (ratio > ratioBound)
        {
            skinnyTets.push_back(tetId);
        }
    }

    return skinnyTets;
}

std::vector<ConstrainedSubsegment3D> MeshOperations3D::findEncroachingSubsegments(
    const Point3D& point,
    const std::vector<ConstrainedSubsegment3D>& subsegments) const
{
    std::vector<ConstrainedSubsegment3D> encroached;
    ConstraintChecker3D checker(meshData_);

    for (const auto& subsegment : subsegments)
    {
        if (checker.isSubsegmentEncroached(subsegment, point))
        {
            encroached.push_back(subsegment);
        }
    }

    return encroached;
}

std::vector<ConstrainedSubfacet3D> MeshOperations3D::findEncroachingSubfacets(
    const Point3D& point,
    const std::vector<ConstrainedSubfacet3D>& subfacets) const
{
    std::vector<ConstrainedSubfacet3D> encroached;
    ConstraintChecker3D checker(meshData_);

    for (const auto& subfacet : subfacets)
    {
        if (checker.isSubfacetEncroached(subfacet, point))
        {
            encroached.push_back(subfacet);
        }
    }

    return encroached;
}

std::array<size_t, 3> MeshOperations3D::makeTriangleKey(size_t a, size_t b, size_t c)
{
    // Sort the three node IDs to create a canonical ordering
    std::array<size_t, 3> key = {a, b, c};
    std::sort(key.begin(), key.end());
    return key;
}

} // namespace Meshing
