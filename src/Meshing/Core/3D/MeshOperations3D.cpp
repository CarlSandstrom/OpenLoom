#include "MeshOperations3D.h"
#include "ElementGeometry3D.h"
#include "Geometry/3D/Base/IEdge3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "GeometryUtilities3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Meshing/Data/3D/Node3D.h"
#include "Meshing/Data/Base/MeshConnectivity.h"
#include "spdlog/spdlog.h"
#include <algorithm>
#include <map>
#include <unordered_set>

namespace Meshing
{

MeshOperations3D::MeshOperations3D(MeshData3D& meshData) :
    meshData_(meshData),
    mutator_(std::make_unique<MeshMutator3D>(meshData))
{
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
    // TODO: Implement flood fill algorithm to classify and remove exterior tetrahedra
    // This is similar to the 2D version but operates on tetrahedral mesh
    // For now, this is a placeholder that doesn't remove anything
    spdlog::info("MeshOperations3D::classifyTetrahedraInteriorExterior: Not yet implemented");
}

std::array<size_t, 3> MeshOperations3D::makeTriangleKey(size_t a, size_t b, size_t c)
{
    // Sort the three node IDs to create a canonical ordering
    std::array<size_t, 3> key = {a, b, c};
    std::sort(key.begin(), key.end());
    return key;
}

} // namespace Meshing
