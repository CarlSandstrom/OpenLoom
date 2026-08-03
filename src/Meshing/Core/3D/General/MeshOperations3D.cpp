#include "Meshing/Core/3D/General/MeshOperations3D.h"
#include "Common/Exceptions/MeshException.h"
#include "Geometry/3D/Base/IEdge3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Core/3D/General/GeometryUtilities3D.h"
#include "Meshing/Core/3D/General/RobustPredicates3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Meshing/Data/3D/Node3D.h"
#include "spdlog/spdlog.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace Meshing
{

namespace
{

// Whether v lies exactly in the plane through p0, p1, p2 -- i.e. whether the
// tetrahedron (v, p0, p1, p2) would have zero volume. Exact (via
// RobustPredicates3D), not tolerance-based: growCavityThroughCoplanarFaces()
// and retriangulate() below must agree on this with insertVertexBowyerWatson's
// in-sphere test (also exact, same module) about which faces are degenerate —
// a tolerance-based coplanarity check that disagrees with an exact in-sphere
// test on borderline cases is exactly what produced inconsistent cavities
// (a boundary face with only one neighboring tetrahedron) on nearly-planar
// boundary curves (see OPE-159/OPE-138).
bool isCoplanarWithFace(const Point3D& v, const Point3D& p0, const Point3D& p1, const Point3D& p2)
{
    return RobustPredicates3D::orientationSign(p0, p1, p2, v) == 0;
}

} // namespace

MeshOperations3D::MeshOperations3D(MeshData3D& meshData) :
    meshData_(meshData),
    queries_(meshData),
    mutator_(std::make_unique<MeshMutator3D>(meshData))
{
}

std::array<size_t, 4> MeshOperations3D::createBoundingTetrahedron(const std::vector<Point3D>& points)
{
    if (points.empty())
    {
        OPENLOOM_THROW_CODE(OpenLoom::MeshException,
                         OpenLoom::MeshException::ErrorCode::INVALID_OPERATION,
                         "createBoundingTetrahedron called with empty point list");
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
    double margin = maxDim * 10.0;               // Large margin for numerical stability

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

    mutator_->setBoundingNodeIds({id0, id1, id2, id3});

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
            if (boundingSet.contains(nodeId))
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

    mutator_->clearBoundingNodeIds();

    spdlog::info("MeshOperations3D::removeBoundingTetrahedron: Removed {} tetrahedra and 4 bounding nodes",
                 tetsToRemove.size());
}

size_t MeshOperations3D::insertVertexBowyerWatson(const Point3D& point,
                                                  const std::vector<std::string>& geometryIds)
{
    // Find conflicting tetrahedra (those whose circumsphere contains the point)
    std::vector<size_t> conflicting = queries_.findConflictingTetrahedra(point);

    if (conflicting.empty())
    {
        spdlog::warn("MeshOperations3D::insertVertexBowyerWatson: No conflicting tetrahedra found");
        // Just add the vertex without removing any tetrahedra
        size_t nodeId = mutator_->addNode(point);
        return nodeId;
    }

    // Extend the cavity through any boundary face coplanar with the new
    // vertex, so retriangulate() never has to fan onto a degenerate face.
    conflicting = growCavityThroughCoplanarFaces(point, std::move(conflicting));

    // Find the cavity boundary
    std::vector<std::array<size_t, 3>> boundary = queries_.findCavityBoundary(conflicting);

    // Remove conflicting tetrahedra
    for (size_t tetId : conflicting)
    {
        mutator_->removeElement(tetId);
    }

    // Insert the new vertex (use boundary node if geometry IDs provided)
    size_t nodeId;
    if (!geometryIds.empty())
    {
        nodeId = mutator_->addBoundaryNode(point, geometryIds);
    }
    else
    {
        nodeId = mutator_->addNode(point);
    }

    // Retriangulate the cavity with the new vertex
    retriangulate(nodeId, boundary);

    return nodeId;
}

std::vector<size_t> MeshOperations3D::growCavityThroughCoplanarFaces(
    const Point3D& point, std::vector<size_t> conflicting) const
{
    std::unordered_set<size_t> conflictingSet(conflicting.begin(), conflicting.end());

    bool grew = true;
    while (grew)
    {
        grew = false;

        for (const auto& face : queries_.findCavityBoundary(conflicting))
        {
            const Node3D* n0 = meshData_.getNode(face[0]);
            const Node3D* n1 = meshData_.getNode(face[1]);
            const Node3D* n2 = meshData_.getNode(face[2]);
            if (!n0 || !n1 || !n2)
                continue;

            if (!isCoplanarWithFace(point, n0->getCoordinates(), n1->getCoordinates(), n2->getCoordinates()))
                continue;

            const auto touching = queries_.findTetrahedraWithFace(face[0], face[1], face[2]);
            if (touching.size() < 2)
            {
                OPENLOOM_THROW_CODE(OpenLoom::MeshException,
                                 OpenLoom::MeshException::ErrorCode::INVALID_TOPOLOGY,
                                 "growCavityThroughCoplanarFaces: cavity boundary face is "
                                 "coplanar with the inserted vertex and has no neighboring "
                                 "tetrahedron to extend into");
            }

            const auto neighborIt = std::find_if(touching.begin(), touching.end(),
                [&conflictingSet](size_t tetId) { return !conflictingSet.contains(tetId); });

            // Both tetrahedra touching this face are already in the cavity (pulled in
            // while processing another coplanar face earlier in this pass) -- already
            // resolved, nothing more to do for this face.
            if (neighborIt == touching.end())
                continue;

            conflictingSet.insert(*neighborIt);
            conflicting.push_back(*neighborIt);
            grew = true;
        }
    }

    return conflicting;
}

void MeshOperations3D::retriangulate(size_t vertexNodeId,
                                     const std::vector<std::array<size_t, 3>>& boundary)
{
    const Node3D* vertexNode = meshData_.getNode(vertexNodeId);
    if (!vertexNode)
    {
        return;
    }
    const Point3D& v = vertexNode->getCoordinates();

    // Create a new tetrahedron for each boundary face, ensuring positive orientation
    for (const auto& face : boundary)
    {
        const Node3D* n0 = meshData_.getNode(face[0]);
        const Node3D* n1 = meshData_.getNode(face[1]);
        const Node3D* n2 = meshData_.getNode(face[2]);
        if (!n0 || !n1 || !n2)
        {
            continue;
        }

        const Point3D& p0 = n0->getCoordinates();
        const Point3D& p1 = n1->getCoordinates();
        const Point3D& p2 = n2->getCoordinates();

        // growCavityThroughCoplanarFaces() is responsible for ensuring no boundary
        // face reaches here coplanar with v; treat it as a programming error rather
        // than silently dropping the face, which would leave a hole in the mesh.
        if (isCoplanarWithFace(v, p0, p1, p2))
        {
            OPENLOOM_THROW_CODE(OpenLoom::MeshException,
                             OpenLoom::MeshException::ErrorCode::INVALID_TOPOLOGY,
                             "retriangulate: cavity boundary face is coplanar with the "
                             "inserted vertex; growCavityThroughCoplanarFaces should have "
                             "extended the cavity past it");
        }

        const Point3D faceNormal = (p1 - p0).cross(p2 - p0);
        const double signedVolume = faceNormal.dot(v - p0);

        // signedVolume is the signed volume of (p0, p1, p2, v) in that vertex
        // order (by the scalar triple product's cyclic invariance, faceNormal
        // . (v - p0) == (p1 - p0) . ((p2 - p0) x (v - p0))). Store the node
        // IDs in that same (face..., vertex) order so a >= 0 signedVolume
        // here really does mean a positive-orientation TetrahedralElement
        // under MeshVerifier3D::computeSignedVolume's convention -- storing
        // vertex first instead (an odd permutation) would silently negate it.
        std::array<size_t, 4> nodeIds;
        if (signedVolume >= 0.0)
        {
            nodeIds = {face[0], face[1], face[2], vertexNodeId};
        }
        else
        {
            // Flip face winding to get positive orientation
            nodeIds = {face[0], face[2], face[1], vertexNodeId};
        }

        auto tet = std::make_unique<TetrahedralElement>(nodeIds);
        mutator_->addElement(std::move(tet));
    }
}

std::optional<std::pair<size_t, size_t>>
MeshOperations3D::splitConstrainedSubsegment(size_t segmentId,
                                             const Geometry3D::IEdge3D& parentEdge)
{
    const CurveSegment& segment = meshData_.getCurveSegmentManager().getSegment(segmentId);

    const auto* node1 = meshData_.getNode(segment.nodeId1);
    const auto* node2 = meshData_.getNode(segment.nodeId2);

    if (!node1 || !node2)
    {
        spdlog::error("MeshOperations3D::splitConstrainedSubsegment: Invalid node IDs");
        return std::nullopt;
    }

    Point3D midpoint = (node1->getCoordinates() + node2->getCoordinates()) * 0.5;

    if (segment.tStart != segment.tEnd)
    {
        double tMid = (segment.tStart + segment.tEnd) * 0.5;
        midpoint = parentEdge.getPoint(tMid);
    }

    size_t midNodeId = insertVertexBowyerWatson(midpoint, {segment.edgeId});

    double tMid = (segment.tStart + segment.tEnd) * 0.5;
    auto [segmentId1, segmentId2] = mutator_->splitCurveSegment(segmentId, midNodeId, tMid);

    return std::make_pair(segmentId1, segmentId2);
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
    size_t centerNodeId = insertVertexBowyerWatson(circumcenter, {subfacet.geometryId});

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

} // namespace Meshing
