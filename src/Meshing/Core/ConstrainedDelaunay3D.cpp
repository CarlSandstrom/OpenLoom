#include "ConstrainedDelaunay3D.h"

#include "Export/VtkExporter.h"
#include "Meshing/Data/TetrahedralElement.h"
#include "spdlog/spdlog.h"
#include <algorithm>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>

namespace Meshing
{

ConstrainedDelaunay3D::ConstrainedDelaunay3D(MeshingContext3D& context) :
    computer_(context.getMeshData()),
    context_(context),
    meshData_(context.getMeshData()),
    operations_(context.getOperations())
{
}

void ConstrainedDelaunay3D::initialize(const std::vector<Point3D>& points)
{
    superNodeIds_.clear();
    activeTetrahedra_.clear();

    if (points.size() < 4)
    {
        throw std::runtime_error("Need at least 4 points for 3D triangulation");
    }

    createSuperTetrahedron(points);

    Export::VtkExporter exporter;
    size_t iteration = 0U;
    for (const auto& p : points)
    {
        insertVertex(p);

        std::ostringstream fileName;
        fileName << "mesh_" << std::setw(3) << std::setfill('0') << iteration << ".vtu";
        exporter.writeVtu(meshData_, fileName.str());
        ++iteration;
    }

    removeSuperTetrahedron();
}

void ConstrainedDelaunay3D::insertVertex(const Point3D& point)
{
    const size_t nodeId = operations_.addNode(point);

    SPDLOG_INFO("Inserting vertex at ({}, {}, {}) as node ID {}",
                point.x(), point.y(), point.z(), nodeId);

    const std::vector<size_t> conflicting = findConflictingTetrahedra(point);
    if (conflicting.empty())
    {
        throw std::runtime_error("No conflicting tetrahedra found - point outside mesh?");
    }

    const std::vector<std::array<size_t, 3>> boundary = findCavityBoundary(conflicting);

    for (size_t elementId : conflicting)
    {
        operations_.removeElement(elementId);
        activeTetrahedra_.erase(elementId);
    }

    retriangulate(nodeId, boundary);
}

bool ConstrainedDelaunay3D::isElementActive(size_t elementId) const
{
    return activeTetrahedra_.find(elementId) != activeTetrahedra_.end();
}

const TetrahedralElement* ConstrainedDelaunay3D::getTetrahedralElement(size_t elementId) const
{
    const auto* element = meshData_.getElement(elementId);
    return dynamic_cast<const TetrahedralElement*>(element);
}

std::vector<size_t> ConstrainedDelaunay3D::findConflictingTetrahedra(const Point3D& p) const
{
    std::vector<size_t> conflicting;
    conflicting.reserve(activeTetrahedra_.size());

    for (size_t elementId : activeTetrahedra_)
    {
        const auto* element = getTetrahedralElement(elementId);
        if (element == nullptr)
        {
            continue;
        }

        if (computer_.getIsPointInsideCircumscribingSphere(*element, p))
        {
            conflicting.push_back(elementId);
        }
    }

    return conflicting;
}

std::vector<std::array<size_t, 3>> ConstrainedDelaunay3D::findCavityBoundary(
    const std::vector<size_t>& conflicting) const
{
    std::map<std::array<size_t, 3>, int> faceCount;
    std::map<std::array<size_t, 3>, std::array<size_t, 3>> faceLookup;

    for (size_t elementId : conflicting)
    {
        const auto* element = getTetrahedralElement(elementId);
        if (element == nullptr)
        {
            continue;
        }

        for (const auto& face : element->getFaces())
        {
            auto sortedFace = face;
            std::sort(sortedFace.begin(), sortedFace.end());
            faceCount[sortedFace]++;
            faceLookup.emplace(sortedFace, face);
        }
    }

    std::vector<std::array<size_t, 3>> boundary;
    boundary.reserve(faceCount.size());

    for (const auto& [face, count] : faceCount)
    {
        if (count == 1)
        {
            boundary.push_back(faceLookup.at(face));
        }
    }

    return boundary;
}

void ConstrainedDelaunay3D::retriangulate(
    size_t vertexNodeId, const std::vector<std::array<size_t, 3>>& boundary)
{
    for (const auto& face : boundary)
    {
        std::array<size_t, 4> nodeIds = {vertexNodeId, face[0], face[1], face[2]};
        auto element = std::make_unique<TetrahedralElement>(nodeIds);
        const size_t elementId = operations_.addElement(std::move(element));
        activeTetrahedra_.insert(elementId);
    }
}

void ConstrainedDelaunay3D::createSuperTetrahedron(const std::vector<Point3D>& points)
{
    double minX = std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double minZ = std::numeric_limits<double>::max();
    double maxX = -std::numeric_limits<double>::max();
    double maxY = -std::numeric_limits<double>::max();
    double maxZ = -std::numeric_limits<double>::max();

    for (const auto& p : points)
    {
        minX = std::min(minX, p.x());
        minY = std::min(minY, p.y());
        minZ = std::min(minZ, p.z());
        maxX = std::max(maxX, p.x());
        maxY = std::max(maxY, p.y());
        maxZ = std::max(maxZ, p.z());
    }

    const double dx = maxX - minX;
    const double dy = maxY - minY;
    const double dz = maxZ - minZ;
    const double dmax = std::max({dx, dy, dz});

    const double midX = (minX + maxX) * 0.5;
    const double midY = (minY + maxY) * 0.5;
    const double midZ = (minZ + maxZ) * 0.5;

    const double scale = 10.0 * dmax;

    superNodeIds_.push_back(operations_.addNode(Point3D(midX - scale, midY - scale, midZ - scale)));
    superNodeIds_.push_back(operations_.addNode(Point3D(midX + scale, midY - scale, midZ - scale)));
    superNodeIds_.push_back(operations_.addNode(Point3D(midX, midY + scale, midZ - scale)));
    superNodeIds_.push_back(operations_.addNode(Point3D(midX, midY, midZ + scale)));

    std::array<size_t, 4> nodeIds = {superNodeIds_[0], superNodeIds_[1], superNodeIds_[2], superNodeIds_[3]};
    auto element = std::make_unique<TetrahedralElement>(nodeIds);
    const size_t elementId = operations_.addElement(std::move(element));
    activeTetrahedra_.insert(elementId);
}

void ConstrainedDelaunay3D::removeSuperTetrahedron()
{
    if (superNodeIds_.empty())
    {
        return;
    }

    const std::unordered_set<size_t> superNodeSet(superNodeIds_.begin(), superNodeIds_.end());

    std::vector<size_t> elementsToRemove;
    for (size_t elementId : activeTetrahedra_)
    {
        const auto* element = getTetrahedralElement(elementId);
        if (element == nullptr)
        {
            continue;
        }

        for (size_t nodeId : element->getNodeIds())
        {
            if (superNodeSet.count(nodeId) > 0U)
            {
                elementsToRemove.push_back(elementId);
                break;
            }
        }
    }

    for (size_t elementId : elementsToRemove)
    {
        operations_.removeElement(elementId);
        activeTetrahedra_.erase(elementId);
    }

    for (size_t nodeId : superNodeIds_)
    {
        operations_.removeNode(nodeId);
    }

    superNodeIds_.clear();
}

bool ConstrainedDelaunay3D::isDelaunay() const
{
    for (size_t elementId : activeTetrahedra_)
    {
        const auto* element = getTetrahedralElement(elementId);
        if (element == nullptr)
        {
            continue;
        }

        const auto sphere = computer_.getCircumscribingSphere(*element);
        if (!sphere)
        {
            continue;
        }

        for (const auto& [nodeId, nodePtr] : meshData_.getNodes())
        {
            if (element->hasNode(nodeId))
            {
                continue;
            }

            if (computer_.getIsPointInsideCircumscribingSphere(*sphere, nodePtr->getCoordinates()))
            {
                return false;
            }
        }
    }

    return true;
}

void ConstrainedDelaunay3D::generateConstrained(size_t samplesPerEdge,
                                                size_t samplesPerSurface)
{
    SPDLOG_INFO("========================================");
    SPDLOG_INFO("Starting CONSTRAINED Delaunay (with FACETS)");
    SPDLOG_INFO("========================================");

    // Step 1: Insert corner nodes
    insertCornerNodes(context_.getTopology(), context_.getGeometry());

    // Step 2: Insert edge nodes and build segment constraints
    insertEdgeNodes(context_.getTopology(), context_.getGeometry(), samplesPerEdge);

    // Step 3: Triangulate surfaces and build facet constraints
    triangulateSurfaces(context_.getTopology(), context_.getGeometry(), samplesPerSurface);

    // Step 4: Collect all points for initial triangulation
    std::vector<Point3D> allPoints;
    allPoints.reserve(meshData_.getNodeCount());

    for (const auto& [nodeId, nodePtr] : meshData_.getNodes())
    {
        allPoints.push_back(nodePtr->getCoordinates());
    }

    // Step 5: Create initial unconstrained Delaunay triangulation
    SPDLOG_INFO("Creating initial Delaunay with {} points", allPoints.size());
    initialize(allPoints);

    // Step 6: Recover all constraint segments
    SPDLOG_INFO("Recovering {} constraint segments", constraints_.getSegmentCount());
    recoverSegments();

    // Step 7: Recover all constraint facets
    SPDLOG_INFO("Recovering {} constraint facets ({} subfacets)",
                constraints_.getFacetCount(), constraints_.getTotalSubfacetCount());
    recoverFacets();

    SPDLOG_INFO("========================================");
    SPDLOG_INFO("Constrained Delaunay COMPLETE");
    SPDLOG_INFO("  Final: {} nodes, {} tets",
                meshData_.getNodeCount(), meshData_.getElementCount());
    SPDLOG_INFO("========================================");
}

void ConstrainedDelaunay3D::insertCornerNodes(const Topology3D::Topology3D& topology,
                                              const Geometry3D::GeometryCollection3D& geometry)
{
    for (const auto& cornerId : topology.getAllCornerIds())
    {
        auto* corner = geometry.getCorner(cornerId);
        Point3D point = corner->getPoint();

        size_t nodeId = operations_.addNode(point);
        topologyToNodeId_[cornerId] = nodeId;

        SPDLOG_DEBUG("Corner {} → node {} at ({:.2f}, {:.2f}, {:.2f})",
                     cornerId, nodeId, point.x(), point.y(), point.z());
    }

    SPDLOG_INFO("✓ Inserted {} corner nodes", topology.getAllCornerIds().size());
}

void ConstrainedDelaunay3D::insertEdgeNodes(const Topology3D::Topology3D& topology,
                                            const Geometry3D::GeometryCollection3D& geometry,
                                            size_t samplesPerEdge)
{
    for (const auto& edgeId : topology.getAllEdgeIds())
    {
        const auto& topoEdge = topology.getEdge(edgeId);
        auto* geomEdge = geometry.getEdge(edgeId);

        size_t startNodeId = topologyToNodeId_[topoEdge.getStartCornerId()];
        size_t endNodeId = topologyToNodeId_[topoEdge.getEndCornerId()];

        auto [tMin, tMax] = geomEdge->getParameterBounds();
        std::vector<size_t> edgeNodeIds;
        edgeNodeIds.push_back(startNodeId);

        // Insert intermediate nodes
        for (size_t i = 1; i < samplesPerEdge; ++i)
        {
            double t = tMin + (tMax - tMin) * i / static_cast<double>(samplesPerEdge);
            Point3D point = geomEdge->getPoint(t);
            size_t nodeId = operations_.addNode(point);
            edgeNodeIds.push_back(nodeId);
        }

        edgeNodeIds.push_back(endNodeId);

        // Create constraint segments
        for (size_t i = 0; i < edgeNodeIds.size() - 1; ++i)
        {
            constraints_.segments.emplace_back(edgeNodeIds[i], edgeNodeIds[i + 1], edgeId);
        }
    }

    SPDLOG_INFO("✓ Created {} constraint segments from {} edges",
                constraints_.getSegmentCount(), topology.getAllEdgeIds().size());
}

void ConstrainedDelaunay3D::triangulateSurfaces(const Topology3D::Topology3D& topology,
                                                const Geometry3D::GeometryCollection3D& geometry,
                                                size_t samplesPerSurface)
{
    for (const auto& surfaceId : topology.getAllSurfaceIds())
    {
        const auto& topoSurface = topology.getSurface(surfaceId);
        auto* geomSurface = geometry.getSurface(surfaceId);

        ConstrainedFacet facet(surfaceId);

        // Collect boundary node IDs from corners
        std::vector<size_t> boundaryNodeIds;
        for (const auto& cornerId : topoSurface.getCornerIds())
        {
            boundaryNodeIds.push_back(topologyToNodeId_[cornerId]);
        }

        facet.boundaryNodeIds = boundaryNodeIds;

        // Sample interior points on surface
        std::vector<size_t> interiorNodeIds;
        auto paramBounds = geomSurface->getParameterBounds();

        for (size_t i = 1; i < samplesPerSurface; ++i)
        {
            for (size_t j = 1; j < samplesPerSurface; ++j)
            {
                double u = paramBounds.getUMin() +
                           (paramBounds.getUMax() - paramBounds.getUMin()) * i / samplesPerSurface;
                double v = paramBounds.getVMin() +
                           (paramBounds.getVMax() - paramBounds.getVMin()) * j / samplesPerSurface;

                Point3D point = geomSurface->getPoint(u, v);
                size_t nodeId = operations_.addNode(point);
                interiorNodeIds.push_back(nodeId);
            }
        }

        // Triangulate using constrained Delaunay 2D
        auto subfacets = triangulateSurface2D(boundaryNodeIds, interiorNodeIds, geomSurface);

        for (const auto& tri : subfacets)
        {
            facet.addSubfacet(tri[0], tri[1], tri[2]);
        }

        constraints_.facets.push_back(std::move(facet));

        SPDLOG_DEBUG("Surface {} → {} subfacets ({} interior nodes)",
                     surfaceId, subfacets.size(), interiorNodeIds.size());
    }

    SPDLOG_INFO("✓ Triangulated {} surfaces → {} subfacets",
                constraints_.getFacetCount(), constraints_.getTotalSubfacetCount());
}

std::vector<std::array<size_t, 3>> ConstrainedDelaunay3D::triangulateSurface2D(
    const std::vector<size_t>& boundaryNodeIds,
    const std::vector<size_t>& interiorNodeIds,
    const Geometry3D::ISurface3D* surface)
{
    std::vector<std::array<size_t, 3>> triangles;

    if (boundaryNodeIds.size() < 3)
    {
        return triangles;
    }

    // Build map from node ID to 2D parametric coordinates
    std::unordered_map<size_t, Point2D> nodeCoords2D;

    // Project boundary nodes to 2D
    for (size_t nodeId : boundaryNodeIds)
    {
        const auto* node = meshData_.getNode(nodeId);
        if (node != nullptr)
        {
            Point2D coord2D = surface->projectPoint(node->getCoordinates());
            nodeCoords2D[nodeId] = coord2D;
        }
    }

    // Project interior nodes to 2D
    for (size_t nodeId : interiorNodeIds)
    {
        const auto* node = meshData_.getNode(nodeId);
        if (node != nullptr)
        {
            Point2D coord2D = surface->projectPoint(node->getCoordinates());
            nodeCoords2D[nodeId] = coord2D;
        }
    }

    // Create constrained Delaunay triangulator
    ConstrainedDelaunay2D delaunay2D(nodeCoords2D);

    // Add constraint edges for the boundary (forming a closed loop)
    for (size_t i = 0; i < boundaryNodeIds.size(); ++i)
    {
        size_t nextIdx = (i + 1) % boundaryNodeIds.size();
        delaunay2D.addConstraintEdge(boundaryNodeIds[i], boundaryNodeIds[nextIdx]);
    }

    // Triangulate
    triangles = delaunay2D.triangulate();

    SPDLOG_DEBUG("Constrained Delaunay 2D: {} boundary nodes, {} interior nodes → {} triangles",
                 boundaryNodeIds.size(), interiorNodeIds.size(), triangles.size());

    return triangles;
}

void ConstrainedDelaunay3D::recoverSegments()
{
    size_t recovered = 0;
    size_t alreadyPresent = 0;

    for (const auto& seg : constraints_.segments)
    {
        if (ConstrainedDelaunay3DHelper::segmentExists(*this,
                                                       seg.startNodeId,
                                                       seg.endNodeId,
                                                       activeTetrahedra_,
                                                       satisfiedSegments_))
        {
            satisfiedSegments_.insert(
                ConstrainedDelaunay3DHelper::makeSegmentKey(seg.startNodeId, seg.endNodeId));
            alreadyPresent++;
        }
        else
        {
            forceSegment(seg.startNodeId, seg.endNodeId);
            satisfiedSegments_.insert(
                ConstrainedDelaunay3DHelper::makeSegmentKey(seg.startNodeId, seg.endNodeId));
            recovered++;
        }
    }

    SPDLOG_INFO("✓ Segments: {} present, {} forced", alreadyPresent, recovered);
}

void ConstrainedDelaunay3D::recoverFacets()
{
    size_t recovered = 0;
    size_t alreadyPresent = 0;

    for (const auto& facet : constraints_.facets)
    {
        for (const auto& subfacet : facet.subfacets)
        {
            const auto& n = subfacet.nodeIds;

            if (ConstrainedDelaunay3DHelper::facetExists(*this,
                                                         n[0],
                                                         n[1],
                                                         n[2],
                                                         activeTetrahedra_,
                                                         satisfiedFacets_))
            {
                satisfiedFacets_.insert(
                    ConstrainedDelaunay3DHelper::makeTriangleKey(n[0], n[1], n[2]));
                alreadyPresent++;
            }
            else
            {
                forceFacet(n[0], n[1], n[2]);
                satisfiedFacets_.insert(
                    ConstrainedDelaunay3DHelper::makeTriangleKey(n[0], n[1], n[2]));
                recovered++;
            }
        }
    }

    SPDLOG_INFO("✓ Facets: {} present, {} forced", alreadyPresent, recovered);
}

void ConstrainedDelaunay3D::forceSegment(size_t n1, size_t n2)
{
    auto intersecting = ConstrainedDelaunay3DHelper::findIntersectingTetrahedra(
        *this, meshData_, n1, n2, activeTetrahedra_);
    if (intersecting.empty()) return;

    auto boundary = findCavityBoundary(intersecting);

    for (size_t tetId : intersecting)
    {
        operations_.removeElement(tetId);
        activeTetrahedra_.erase(tetId);
    }

    ConstrainedDelaunay3DHelper::retriangulateCavityWithSegment(
        operations_, activeTetrahedra_, n1, n2, boundary);
}

void ConstrainedDelaunay3D::forceFacet(size_t n0, size_t n1, size_t n2)
{
    auto intersecting = ConstrainedDelaunay3DHelper::findIntersectingTetrahedraForFacet(
        *this, meshData_, n0, n1, n2, activeTetrahedra_);
    if (intersecting.empty()) return;

    SPDLOG_DEBUG("Facet ({},{},{}) intersects {} tets", n0, n1, n2, intersecting.size());

    auto boundary = findCavityBoundary(intersecting);

    for (size_t tetId : intersecting)
    {
        operations_.removeElement(tetId);
        activeTetrahedra_.erase(tetId);
    }

    ConstrainedDelaunay3DHelper::retriangulateCavityWithFacet(
        operations_, activeTetrahedra_, n0, n1, n2, boundary);
}

std::vector<std::array<size_t, 3>> ConstrainedDelaunay3D::triangulateSurfaceWithContext(
    const Geometry3D::ISurface3D* surface,
    const Topology3D::Surface3D& topoSurface,
    size_t samplesPerEdge)
{
    std::vector<std::array<size_t, 3>> triangles;

    if (surface == nullptr)
    {
        return triangles;
    }

    // Create a 2D meshing context from the 3D surface
    auto context2D = MeshingContext2D::fromSurface(
        *surface,
        topoSurface,
        context_.getTopology(),
        context_.getGeometry());

    // Create constrained Delaunay 2D with the context
    ConstrainedDelaunay2D delaunay2D(context2D);

    // Generate constrained triangulation
    delaunay2D.generateConstrained(samplesPerEdge);

    // Extract the triangulation results
    // Note: The nodes are in 2D parametric space; we need to map them back
    // to our 3D mesh node IDs. For now, we return the triangles directly
    // as they use the context's node IDs which are separate from our 3D mesh.

    const auto& meshData2D = delaunay2D.getMeshData2D();
    for (const auto& [elemId, elemPtr] : meshData2D.getElements())
    {
        if (elemPtr != nullptr)
        {
            const auto& nodeIds = elemPtr->getNodeIds();
            if (nodeIds.size() == 3)
            {
                triangles.push_back({nodeIds[0], nodeIds[1], nodeIds[2]});
            }
        }
    }

    SPDLOG_DEBUG("Surface triangulated with context: {} triangles", triangles.size());

    return triangles;
}

} // namespace Meshing
