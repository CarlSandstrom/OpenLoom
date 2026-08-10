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
#include <set>
#include <unordered_set>

namespace Meshing
{

namespace
{

// Whether v lies exactly in the plane through p0, p1, p2 -- i.e. whether the
// tetrahedron (v, p0, p1, p2) would have zero volume. Exact, not
// tolerance-based (see RobustPredicates3D).
bool isCoplanarWithFace(const Point3D& v, const Point3D& p0, const Point3D& p1, const Point3D& p2)
{
    return RobustPredicates3D::orientationSign(p0, p1, p2, v) == 0;
}

// Euler characteristic of the closed surface formed by a set of boundary
// triangles -- V - E + F. A topological sphere (the shape retriangulate()'s
// single-point vertex fan requires -- every boundary face must be "visible"
// via a straight line from the new vertex that stays inside the cavity) is
// always 2. growCavityThroughCoplanarFaces() uses this to detect when
// growing the cavity through one more coplanar face would produce something
// else -- e.g. a solid-torus-shaped cavity (Euler characteristic 0) from
// wrapping all the way around an axis, or a non-manifold boundary (an odd
// characteristic; see OPE-173) -- and declines that growth instead of
// producing a cavity the vertex fan can't validly cover.
int eulerCharacteristic(const std::vector<std::array<size_t, 3>>& boundary)
{
    std::set<size_t> vertices;
    std::set<std::pair<size_t, size_t>> edges;
    for (const auto& face : boundary)
    {
        for (size_t v : face)
            vertices.insert(v);
        for (int i = 0; i < 3; ++i)
        {
            size_t a = face[static_cast<size_t>(i)];
            size_t b = face[static_cast<size_t>((i + 1) % 3)];
            if (a > b)
                std::swap(a, b);
            edges.insert({a, b});
        }
    }
    return static_cast<int>(vertices.size()) - static_cast<int>(edges.size()) + static_cast<int>(boundary.size());
}

// A coplanar cavity boundary face that growCavityThroughCoplanarFaces()
// declined to grow through (see its comment): the two tetrahedra that used
// to share it -- one on the conflicting (removed) side, one on the kept
// (neighbor) side -- need a local 3-way split around the new vertex instead
// of a plain fan. Captured before either tetrahedron is removed, since
// findOppositeVertex() needs both to still exist.
struct CoplanarFaceSplit
{
    std::array<size_t, 3> face;
    size_t apexA;
    size_t neighborTetId;
    size_t apexB;
};

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
                                                  const std::vector<std::string>& geometryIds,
                                                  double weight)
{
    return insertVertexBowyerWatson(point, queries_.findConflictingTetrahedra(point, weight), geometryIds, weight);
}

size_t MeshOperations3D::insertVertexBowyerWatson(const Point3D& point,
                                                  std::vector<size_t> conflicting,
                                                  const std::vector<std::string>& geometryIds,
                                                  double weight)
{
    if (conflicting.empty())
    {
        spdlog::warn("MeshOperations3D::insertVertexBowyerWatson: No conflicting tetrahedra found");
        // Just add the vertex without removing any tetrahedra
        size_t nodeId = mutator_->addNode(point, weight);
        return nodeId;
    }

    conflicting = growCavityThroughCoplanarFaces(point, std::move(conflicting));
    const std::unordered_set<size_t> conflictingSet(conflicting.begin(), conflicting.end());

    const std::vector<std::array<size_t, 3>> boundary = queries_.findCavityBoundary(conflicting);

    // Partition the boundary: growCavityThroughCoplanarFaces() only grows
    // through a coplanar face when doing so keeps the cavity a topological
    // sphere (OPE-173); any coplanar face it declined to grow through is
    // still on the boundary here and needs a local 3-way split around the
    // new vertex instead of retriangulate()'s single-point fan, which would
    // otherwise create a zero-volume tetrahedron.
    std::vector<std::array<size_t, 3>> normalFaces;
    std::vector<CoplanarFaceSplit> splits;
    for (const auto& face : boundary)
    {
        const Node3D* n0 = meshData_.getNode(face[0]);
        const Node3D* n1 = meshData_.getNode(face[1]);
        const Node3D* n2 = meshData_.getNode(face[2]);
        if (!n0 || !n1 || !n2 ||
            !isCoplanarWithFace(point, n0->getCoordinates(), n1->getCoordinates(), n2->getCoordinates()))
        {
            normalFaces.push_back(face);
            continue;
        }

        const auto touching = queries_.findTetrahedraWithFace(face[0], face[1], face[2]);
        const auto conflictingSideIt = std::find_if(
            touching.begin(), touching.end(), [&](size_t tetId)
            { return conflictingSet.contains(tetId); });
        const auto neighborIt = std::find_if(
            touching.begin(), touching.end(), [&](size_t tetId)
            { return !conflictingSet.contains(tetId); });

        if (conflictingSideIt == touching.end() || neighborIt == touching.end())
        {
            // No usable neighbor to split against -- a genuine domain
            // boundary (nothing on the other side) or some other unusual
            // configuration. Fall back to a plain fan rather than fail the
            // insertion.
            normalFaces.push_back(face);
            continue;
        }

        const size_t apexA = queries_.findOppositeVertex(*conflictingSideIt, face[0], face[1], face[2]);
        const size_t apexB = queries_.findOppositeVertex(*neighborIt, face[0], face[1], face[2]);
        if (apexA == SIZE_MAX || apexB == SIZE_MAX)
        {
            normalFaces.push_back(face);
            continue;
        }

        splits.push_back({face, apexA, *neighborIt, apexB});
    }

    // Remove conflicting tetrahedra, plus each split's neighbor tetrahedron
    // (captured above, before removal, since it isn't in `conflicting`).
    for (size_t tetId : conflicting)
    {
        mutator_->removeElement(tetId);
    }
    for (const auto& split : splits)
    {
        mutator_->removeElement(split.neighborTetId);
    }

    // Insert the new vertex (use boundary node if geometry IDs provided)
    size_t nodeId;
    if (!geometryIds.empty())
    {
        nodeId = mutator_->addBoundaryNode(point, geometryIds, weight);
    }
    else
    {
        nodeId = mutator_->addNode(point, weight);
    }

    retriangulate(nodeId, normalFaces);
    for (const auto& split : splits)
    {
        splitCoplanarBoundaryFace(nodeId, split.face, split.apexA, split.apexB);
    }

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
            const auto neighborIt = std::find_if(touching.begin(), touching.end(),
                                                 [&conflictingSet](size_t tetId)
                                                 { return !conflictingSet.contains(tetId); });

            // Either there's no neighbor to extend into (a genuine domain
            // boundary) or both sides are already conflicting -- either way,
            // nothing more to do for this face here; insertVertexBowyerWatson()
            // handles whatever's left on the boundary afterward.
            if (neighborIt == touching.end())
                continue;

            // Tentatively grow through this face and check whether the
            // result is still a topological sphere before committing
            // (OPE-173): growing indiscriminately can wrap the cavity into a
            // non-simply-connected shape (e.g. a solid-torus-shaped cavity
            // from wrapping all the way around an axis), which
            // retriangulate()'s single-point vertex fan cannot validly
            // cover. If committing would break that, leave this face
            // unresolved -- insertVertexBowyerWatson() falls back to a local
            // coplanar-face split for it instead.
            std::vector<size_t> tentative = conflicting;
            tentative.push_back(*neighborIt);
            if (eulerCharacteristic(queries_.findCavityBoundary(tentative)) != 2)
                continue;

            conflictingSet.insert(*neighborIt);
            conflicting.push_back(*neighborIt);
            grew = true;
        }
    }

    return conflicting;
}

void MeshOperations3D::addOrientedTetrahedron(const std::array<size_t, 3>& face, size_t apexNodeId)
{
    const Node3D* n0 = meshData_.getNode(face[0]);
    const Node3D* n1 = meshData_.getNode(face[1]);
    const Node3D* n2 = meshData_.getNode(face[2]);
    const Node3D* apexNode = meshData_.getNode(apexNodeId);
    if (!n0 || !n1 || !n2 || !apexNode)
    {
        return;
    }

    const Point3D& p0 = n0->getCoordinates();
    const Point3D& p1 = n1->getCoordinates();
    const Point3D& p2 = n2->getCoordinates();
    const Point3D& apex = apexNode->getCoordinates();

    const Point3D faceNormal = (p1 - p0).cross(p2 - p0);
    const double signedVolume = faceNormal.dot(apex - p0);

    // signedVolume is the signed volume of (p0, p1, p2, apex) in that vertex
    // order (by the scalar triple product's cyclic invariance, faceNormal
    // . (apex - p0) == (p1 - p0) . ((p2 - p0) x (apex - p0))). Store the node
    // IDs in that same (face..., apex) order so a >= 0 signedVolume here
    // really does mean a positive-orientation TetrahedralElement under
    // MeshVerifier3D::computeSignedVolume's convention -- storing the apex
    // first instead (an odd permutation) would silently negate it.
    std::array<size_t, 4> nodeIds;
    if (signedVolume >= 0.0)
    {
        nodeIds = {face[0], face[1], face[2], apexNodeId};
    }
    else
    {
        // Flip face winding to get positive orientation
        nodeIds = {face[0], face[2], face[1], apexNodeId};
    }

    mutator_->addElement(std::make_unique<TetrahedralElement>(nodeIds));
}

void MeshOperations3D::retriangulate(size_t vertexNodeId,
                                     const std::vector<std::array<size_t, 3>>& boundary)
{
    for (const auto& face : boundary)
    {
        addOrientedTetrahedron(face, vertexNodeId);
    }
}

void MeshOperations3D::splitCoplanarBoundaryFace(size_t vertexNodeId,
                                                 const std::array<size_t, 3>& face,
                                                 size_t apexA,
                                                 size_t apexB)
{
    // face and vertexNodeId are coplanar (that's why this face reached here
    // instead of retriangulate()): fanning a single tetrahedron onto face
    // from either apex would be zero-volume. Instead split face into 3
    // sub-triangles around vertexNodeId and fan each to BOTH apexes -- the
    // standard "pyramid subdivision" of the two tetrahedra (face, apexA) and
    // (face, apexB) that used to share this face, now sharing vertexNodeId
    // as an interior point of their common base instead.
    const std::array<std::array<size_t, 3>, 3> subFaces = {{
        {face[0], face[1], vertexNodeId},
        {face[1], face[2], vertexNodeId},
        {face[2], face[0], vertexNodeId},
    }};
    for (const auto& subFace : subFaces)
    {
        addOrientedTetrahedron(subFace, apexA);
        addOrientedTetrahedron(subFace, apexB);
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
