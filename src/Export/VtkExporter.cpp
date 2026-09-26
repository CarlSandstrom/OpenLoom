#include "VtkExporter.h"

#include "Meshing/Core/3D/General/DiscretizationResult3D.h"
#include "Meshing/Core/3D/General/GeometryStructures3D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/Node2D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/Node3D.h"
#include "Meshing/Data/3D/SurfaceMesh3D.h"
#include "Meshing/Data/3D/VolumeMesh3D.h"
#include "Meshing/Data/Base/IElement.h"
#include "Meshing/Data/CurveSegmentManager.h"
#include "VtkGrid.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <numeric>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <variant>
#include <vector>

namespace Export
{

namespace
{

const char* vtkTypeName(const std::vector<int>&) { return "Int32"; }
const char* vtkTypeName(const std::vector<std::size_t>&) { return "UInt64"; }
const char* vtkTypeName(const std::vector<double>&) { return "Float64"; }

void writeField(std::ostream& os, const VtkField& field)
{
    std::visit(
        [&os, &field](const auto& values)
        {
            os << "        <DataArray type=\"" << vtkTypeName(values) << "\" Name=\"" << field.name
               << "\" format=\"ascii\">\n          ";
            for (std::size_t i = 0; i < values.size(); ++i)
                os << values[i] << (i + 1 == values.size() ? "" : " ");
        },
        field.values);
    os << "\n        </DataArray>\n";
}

void writeGridCells(std::ostream& os, const std::vector<VtkCell>& cells)
{
    os << "      <Cells>\n";
    os << "        <DataArray type=\"Int64\" Name=\"connectivity\" format=\"ascii\">\n          ";
    for (const auto& cell : cells)
    {
        for (std::size_t index : cell.pointIndices)
            os << index << ' ';
    }
    os << "\n        </DataArray>\n";

    os << "        <DataArray type=\"Int64\" Name=\"offsets\" format=\"ascii\">\n          ";
    std::size_t runningOffset = 0;
    for (const auto& cell : cells)
    {
        runningOffset += cell.pointIndices.size();
        os << runningOffset << ' ';
    }
    os << "\n        </DataArray>\n";

    os << "        <DataArray type=\"UInt8\" Name=\"types\" format=\"ascii\">\n          ";
    for (const auto& cell : cells)
        os << static_cast<int>(cell.type) << ' ';
    os << "\n        </DataArray>\n";
    os << "      </Cells>\n";
}

VtkCellType cellTypeFor(Meshing::ElementType type)
{
    switch (type)
    {
    case Meshing::ElementType::TETRAHEDRON:
        return VtkCellType::Tetrahedron;
    case Meshing::ElementType::HEXAHEDRON:
        return VtkCellType::Hexahedron;
    case Meshing::ElementType::PRISM:
        return VtkCellType::Wedge;
    case Meshing::ElementType::PYRAMID:
        return VtkCellType::Pyramid;
    case Meshing::ElementType::TRIANGLE:
        return VtkCellType::Triangle;
    case Meshing::ElementType::QUADRILATERAL:
        return VtkCellType::Quadrilateral;
    }
    return VtkCellType::Triangle;
}

Meshing::Point3D toPoint3D(const Meshing::Point3D& point)
{
    return point;
}

Meshing::Point3D toPoint3D(const Meshing::Point2D& point)
{
    return {point.x(), point.y(), 0.0};
}

std::vector<std::size_t> sequence(std::size_t count)
{
    std::vector<std::size_t> values(count);
    std::iota(values.begin(), values.end(), 0);
    return values;
}

// Points given in index order, so each point's NodeID is its index.
VtkGrid gridFromPoints(const std::vector<Meshing::Point3D>& points)
{
    VtkGrid grid;
    grid.points = points;
    grid.nodeIds = sequence(points.size());
    return grid;
}

template <typename Map>
std::vector<std::size_t> sortedIds(const Map& map)
{
    std::vector<std::size_t> ids;
    ids.reserve(map.size());
    for (const auto& entry : map)
        ids.push_back(entry.first);
    std::sort(ids.begin(), ids.end());
    return ids;
}

// A stable 0-based index per distinct surface id, in sorted order, so a
// SurfaceID colour means the same surface across runs.
std::unordered_map<std::string, int> indexSurfaceIds(std::vector<std::string> surfaceIds)
{
    std::sort(surfaceIds.begin(), surfaceIds.end());
    surfaceIds.erase(std::unique(surfaceIds.begin(), surfaceIds.end()), surfaceIds.end());

    std::unordered_map<std::string, int> indexBySurfaceId;
    for (int i = 0; i < static_cast<int>(surfaceIds.size()); ++i)
        indexBySurfaceId[surfaceIds[i]] = i;
    return indexBySurfaceId;
}

// Per-triangle SurfaceID from a per-surface grouping; -1 for a triangle no
// surface claims.
std::vector<int> triangleSurfaceIndices(const std::map<std::string, std::vector<std::size_t>>& groups,
                                        std::size_t triangleCount)
{
    std::vector<std::string> surfaceIds;
    for (const auto& [surfaceId, triangleIds] : groups)
        surfaceIds.push_back(surfaceId);
    const auto indexBySurfaceId = indexSurfaceIds(surfaceIds);

    std::vector<int> indices(triangleCount, -1);
    for (const auto& [surfaceId, triangleIds] : groups)
    {
        for (std::size_t triangleId : triangleIds)
            indices[triangleId] = indexBySurfaceId.at(surfaceId);
    }
    return indices;
}

/// A MeshData2D or MeshData3D laid out as a grid: nodes sorted by id, then
/// every element sorted by id, then every constraint segment as a line.
/// elementIds receives the element ids in cell order.
template <typename MeshData>
VtkGrid gridFromMeshData(const MeshData& mesh, std::vector<std::size_t>& elementIds)
{
    VtkGrid grid;
    grid.nodeIds = sortedIds(mesh.getNodes());
    std::unordered_map<std::size_t, std::size_t> indexByNodeId;
    for (std::size_t i = 0; i < grid.nodeIds.size(); ++i)
    {
        indexByNodeId[grid.nodeIds[i]] = i;
        grid.points.push_back(toPoint3D(mesh.getNode(grid.nodeIds[i])->getCoordinates()));
    }

    elementIds = sortedIds(mesh.getElements());
    for (std::size_t elementId : elementIds)
    {
        const auto* element = mesh.getElement(elementId);
        VtkCell cell{cellTypeFor(element->getType()), {}};
        for (std::size_t nodeId : element->getNodeIds())
            cell.pointIndices.push_back(indexByNodeId.at(nodeId));
        grid.cells.push_back(std::move(cell));
    }

    for (const auto& [segmentId, segment] : mesh.getCurveSegmentManager().getAllSegments())
    {
        grid.cells.push_back(
            {VtkCellType::Line, {indexByNodeId.at(segment.nodeId1), indexByNodeId.at(segment.nodeId2)}});
    }
    return grid;
}

// 0 for the first leadingCount cells and 1 for the rest: how EdgeRole marks
// segments after elements, and IsBoundaryTriangle triangles after tetrahedra.
std::vector<int> flagTrailingCells(std::size_t leadingCount, std::size_t cellCount)
{
    std::vector<int> flags(cellCount, 1);
    std::fill(flags.begin(), flags.begin() + static_cast<std::ptrdiff_t>(leadingCount), 0);
    return flags;
}

// Labels each triangle with the connected region it belongs to, flood-filling
// between constraint edges; empty when the mesh has no constraints.
std::unordered_map<std::size_t, int> computeDomainIds(const Meshing::MeshData2D& mesh)
{
    std::unordered_map<std::size_t, int> domainIds;

    const auto& curveSegmentManager = mesh.getCurveSegmentManager();
    if (curveSegmentManager.empty())
        return domainIds;

    // Edge key helper (canonical ordering)
    using EdgeKey = std::pair<std::size_t, std::size_t>;
    auto makeEdgeKey = [](std::size_t a, std::size_t b) -> EdgeKey
    {
        return a < b ? EdgeKey{a, b} : EdgeKey{b, a};
    };
    struct EdgeKeyHash
    {
        std::size_t operator()(const EdgeKey& k) const
        {
            return std::hash<std::size_t>{}(k.first) ^ (std::hash<std::size_t>{}(k.second) << 1);
        }
    };

    // Collect boundary and all constraint edge keys
    std::unordered_set<EdgeKey, EdgeKeyHash> boundaryEdges;
    std::unordered_set<EdgeKey, EdgeKeyHash> allConstraintEdges;

    for (const auto& [segId, seg] : curveSegmentManager.getAllSegments())
    {
        auto key = makeEdgeKey(seg.nodeId1, seg.nodeId2);
        allConstraintEdges.insert(key);
        if (seg.role == Meshing::ConstraintRole::Boundary)
            boundaryEdges.insert(key);
    }

    // Build edge-to-triangles adjacency map
    std::unordered_map<EdgeKey, std::vector<std::size_t>, EdgeKeyHash> edgeToTriangles;
    for (const auto& [elemId, element] : mesh.getElements())
    {
        const auto& nodeIds = element->getNodeIds();
        for (std::size_t i = 0; i < nodeIds.size(); ++i)
        {
            std::size_t a = nodeIds[i];
            std::size_t b = nodeIds[(i + 1) % nodeIds.size()];
            edgeToTriangles[makeEdgeKey(a, b)].push_back(elemId);
        }
    }

    // Ray casting: test if a point is inside the domain boundary
    auto isInsideDomain = [&](const Meshing::Point2D& point) -> bool
    {
        int crossings = 0;
        for (const auto& [segId, seg] : curveSegmentManager.getAllSegments())
        {
            if (seg.role != Meshing::ConstraintRole::Boundary)
                continue;

            const auto& p1 = mesh.getNode(seg.nodeId1)->getCoordinates();
            const auto& p2 = mesh.getNode(seg.nodeId2)->getCoordinates();

            if ((p1.y() > point.y()) == (p2.y() > point.y()))
                continue;

            double xIntersect = p1.x() + (point.y() - p1.y()) / (p2.y() - p1.y()) * (p2.x() - p1.x());
            if (point.x() < xIntersect)
                crossings++;
        }
        return (crossings % 2) == 1;
    };

    // Find a seed triangle whose centroid is inside the boundary
    std::size_t seedTriangle = 0;
    bool foundSeed = false;
    for (const auto& [elemId, element] : mesh.getElements())
    {
        const auto& nodeIds = element->getNodeIds();
        Meshing::Point2D centroid = Meshing::Point2D::Zero();
        for (std::size_t nid : nodeIds)
        {
            centroid += mesh.getNode(nid)->getCoordinates();
        }
        centroid /= static_cast<double>(nodeIds.size());

        if (isInsideDomain(centroid))
        {
            seedTriangle = elemId;
            foundSeed = true;
            break;
        }
    }

    if (!foundSeed)
        return domainIds;

    // Phase 1: BFS flood fill from seed, stopping at BOUNDARY constraints → interior triangles
    std::unordered_set<std::size_t> interiorTriangles;
    {
        std::queue<std::size_t> queue;
        queue.push(seedTriangle);
        interiorTriangles.insert(seedTriangle);

        while (!queue.empty())
        {
            std::size_t current = queue.front();
            queue.pop();

            const auto& nodeIds = mesh.getElement(current)->getNodeIds();
            for (std::size_t i = 0; i < nodeIds.size(); ++i)
            {
                auto edgeKey = makeEdgeKey(nodeIds[i], nodeIds[(i + 1) % nodeIds.size()]);

                if (boundaryEdges.count(edgeKey))
                    continue;

                auto it = edgeToTriangles.find(edgeKey);
                if (it == edgeToTriangles.end())
                    continue;

                for (std::size_t neighbor : it->second)
                {
                    if (neighbor != current && !interiorTriangles.count(neighbor))
                    {
                        interiorTriangles.insert(neighbor);
                        queue.push(neighbor);
                    }
                }
            }
        }
    }

    // Phase 2: BFS flood fill on interior triangles, stopping at ALL constraints → domain IDs
    std::unordered_map<EdgeKey, std::vector<std::size_t>, EdgeKeyHash> interiorEdgeToTriangles;
    for (std::size_t elemId : interiorTriangles)
    {
        const auto& nodeIds = mesh.getElement(elemId)->getNodeIds();
        for (std::size_t i = 0; i < nodeIds.size(); ++i)
        {
            std::size_t a = nodeIds[i];
            std::size_t b = nodeIds[(i + 1) % nodeIds.size()];
            interiorEdgeToTriangles[makeEdgeKey(a, b)].push_back(elemId);
        }
    }

    int nextDomainId = 0;
    std::unordered_set<std::size_t> assigned;

    // Temporary domain ID assignment (order-dependent, will be remapped below)
    for (std::size_t elemId : interiorTriangles)
    {
        if (assigned.count(elemId))
            continue;

        int currentDomain = nextDomainId++;
        std::queue<std::size_t> queue;
        queue.push(elemId);
        assigned.insert(elemId);
        domainIds[elemId] = currentDomain;

        while (!queue.empty())
        {
            std::size_t current = queue.front();
            queue.pop();

            const auto& nodeIds = mesh.getElement(current)->getNodeIds();
            for (std::size_t i = 0; i < nodeIds.size(); ++i)
            {
                auto edgeKey = makeEdgeKey(nodeIds[i], nodeIds[(i + 1) % nodeIds.size()]);

                if (allConstraintEdges.count(edgeKey))
                    continue;

                auto it = interiorEdgeToTriangles.find(edgeKey);
                if (it == interiorEdgeToTriangles.end())
                    continue;

                for (std::size_t neighbor : it->second)
                {
                    if (neighbor != current && !assigned.count(neighbor))
                    {
                        assigned.insert(neighbor);
                        domainIds[neighbor] = currentDomain;
                        queue.push(neighbor);
                    }
                }
            }
        }
    }

    // Remap domain IDs by centroid position so they are stable across mesh changes.
    // Compute the centroid of each domain, sort by (x, y), and reassign IDs.
    struct DomainInfo
    {
        int originalId;
        double centroidX = 0.0;
        double centroidY = 0.0;
        std::size_t count = 0;
    };

    std::unordered_map<int, DomainInfo> domainInfos;
    for (const auto& [elemId, domain] : domainIds)
    {
        auto& info = domainInfos[domain];
        info.originalId = domain;

        const auto& nodeIds = mesh.getElement(elemId)->getNodeIds();
        Meshing::Point2D centroid = Meshing::Point2D::Zero();
        for (std::size_t nid : nodeIds)
            centroid += mesh.getNode(nid)->getCoordinates();
        centroid /= static_cast<double>(nodeIds.size());

        info.centroidX += centroid.x();
        info.centroidY += centroid.y();
        info.count++;
    }

    std::vector<DomainInfo> sortedDomains;
    sortedDomains.reserve(domainInfos.size());
    for (auto& [id, info] : domainInfos)
    {
        info.centroidX /= static_cast<double>(info.count);
        info.centroidY /= static_cast<double>(info.count);
        sortedDomains.push_back(info);
    }

    std::sort(sortedDomains.begin(), sortedDomains.end(), [](const DomainInfo& a, const DomainInfo& b)
              {
                  constexpr double EPS = 1e-6;
                  if (std::abs(a.centroidX - b.centroidX) > EPS)
                      return a.centroidX < b.centroidX;
                  return a.centroidY < b.centroidY;
              });

    std::unordered_map<int, int> remapping;
    for (int i = 0; i < static_cast<int>(sortedDomains.size()); ++i)
        remapping[sortedDomains[i].originalId] = i;

    for (auto& [elemId, domain] : domainIds)
        domain = remapping[domain];

    return domainIds;
}

} // namespace

bool VtkExporter::exportMesh(const Meshing::MeshData3D& mesh, const std::string& filePath) const
{
    std::vector<std::size_t> elementIds;
    VtkGrid grid = gridFromMeshData(mesh, elementIds);

    std::vector<std::size_t> cellIds = elementIds;
    for (const auto& [segmentId, segment] : mesh.getCurveSegmentManager().getAllSegments())
        cellIds.push_back(segmentId);

    const auto& boundingNodeIds = mesh.getBoundingNodeIds();
    std::vector<int> isSupertet(grid.cells.size(), 0);
    for (std::size_t i = 0; boundingNodeIds && i < elementIds.size(); ++i)
    {
        const auto* element = mesh.getElement(elementIds[i]);
        isSupertet[i] = std::any_of(boundingNodeIds->begin(), boundingNodeIds->end(),
                                    [element](std::size_t nodeId)
                                    { return element->hasNode(nodeId); });
    }

    grid.cellFields.push_back({"ElementID", std::move(cellIds)});
    grid.cellFields.push_back({"EdgeRole", flagTrailingCells(elementIds.size(), grid.cells.size())});
    grid.cellFields.push_back({"IsSupertet", std::move(isSupertet)});
    return writeGrid(grid, filePath);
}

bool VtkExporter::exportMesh(const Meshing::MeshData2D& mesh, const std::string& filePath) const
{
    std::vector<std::size_t> elementIds;
    VtkGrid grid = gridFromMeshData(mesh, elementIds);

    // Constraint segments carry ElementID 0.
    std::vector<std::size_t> cellIds(grid.cells.size(), 0);
    std::copy(elementIds.begin(), elementIds.end(), cellIds.begin());
    grid.cellFields.push_back({"ElementID", std::move(cellIds)});
    grid.cellFields.push_back({"EdgeRole", flagTrailingCells(elementIds.size(), grid.cells.size())});

    const auto domainIds = computeDomainIds(mesh);
    if (!domainIds.empty())
    {
        std::vector<int> domains(grid.cells.size(), -1);
        for (std::size_t i = 0; i < elementIds.size(); ++i)
        {
            const auto found = domainIds.find(elementIds[i]);
            if (found != domainIds.end())
                domains[i] = found->second;
        }
        grid.cellFields.push_back({"DomainID", std::move(domains)});
    }
    return writeGrid(grid, filePath);
}

bool VtkExporter::writeEdgeMesh(const Meshing::DiscretizationResult3D& result, const std::string& filePath) const
{
    VtkGrid grid = gridFromPoints(result.points);

    // EdgeID indexes the topology edges in map order, counting edges too short
    // to contribute a line so the index stays aligned with the edge list.
    std::vector<int> edgeIndices;
    int edgeIndex = 0;
    for (const auto& [edgeId, pointIndices] : result.edgeIdToPointIndicesMap)
    {
        for (std::size_t i = 0; i + 1 < pointIndices.size(); ++i)
        {
            grid.cells.push_back({VtkCellType::Line, {pointIndices[i], pointIndices[i + 1]}});
            edgeIndices.push_back(edgeIndex);
        }
        ++edgeIndex;
    }

    grid.cellFields.push_back({"EdgeID", std::move(edgeIndices)});
    return writeGrid(grid, filePath);
}

bool VtkExporter::writeSurfaceMesh(const Meshing::DiscretizationResult3D& discretization,
                                   const std::vector<Meshing::ConstrainedSubfacet3D>& subfacets,
                                   const std::string& filePath) const
{
    VtkGrid grid = gridFromPoints(discretization.points);

    std::vector<std::string> surfaceIds;
    for (const auto& subfacet : subfacets)
        surfaceIds.push_back(subfacet.geometryId);
    const auto indexBySurfaceId = indexSurfaceIds(surfaceIds);

    std::vector<int> surfaceIndices;
    std::vector<int> constraintRoles;
    for (const auto& subfacet : subfacets)
    {
        grid.cells.push_back({VtkCellType::Triangle, {subfacet.nodeId1, subfacet.nodeId2, subfacet.nodeId3}});
        surfaceIndices.push_back(indexBySurfaceId.at(subfacet.geometryId));
        constraintRoles.push_back(subfacet.role == Meshing::ConstraintRole::Boundary ? 0 : 1);
    }

    grid.cellFields.push_back({"SurfaceID", std::move(surfaceIndices)});
    grid.cellFields.push_back({"ConstraintRole", std::move(constraintRoles)});
    return writeGrid(grid, filePath);
}

bool VtkExporter::writeSurfaceMesh(const Meshing::SurfaceMesh3D& surfaceMesh, const std::string& filePath) const
{
    VtkGrid grid = gridFromPoints(surfaceMesh.nodes);
    for (const auto& triangle : surfaceMesh.triangles)
        grid.cells.push_back({VtkCellType::Triangle, {triangle[0], triangle[1], triangle[2]}});

    grid.cellFields.push_back(
        {"SurfaceID", triangleSurfaceIndices(surfaceMesh.faceTriangleIds, surfaceMesh.triangles.size())});
    return writeGrid(grid, filePath);
}

bool VtkExporter::writeVolumeMesh(const Meshing::VolumeMesh3D& volumeMesh, const std::string& filePath) const
{
    VtkGrid grid = gridFromPoints(volumeMesh.nodes);
    for (const auto& tetrahedron : volumeMesh.tetrahedra)
    {
        grid.cells.push_back(
            {VtkCellType::Tetrahedron, {tetrahedron[0], tetrahedron[1], tetrahedron[2], tetrahedron[3]}});
    }
    for (const auto& triangle : volumeMesh.boundaryTriangles)
        grid.cells.push_back({VtkCellType::Triangle, {triangle[0], triangle[1], triangle[2]}});

    // Tetrahedra have no surface; boundary triangles follow them.
    const std::size_t tetrahedronCount = volumeMesh.tetrahedra.size();
    std::vector<int> surfaceIndices(tetrahedronCount, -1);
    const auto triangleIndices =
        triangleSurfaceIndices(volumeMesh.boundaryFaceTriangleIds, volumeMesh.boundaryTriangles.size());
    surfaceIndices.insert(surfaceIndices.end(), triangleIndices.begin(), triangleIndices.end());

    grid.cellFields.push_back({"SurfaceID", std::move(surfaceIndices)});
    grid.cellFields.push_back({"IsBoundaryTriangle", flagTrailingCells(tetrahedronCount, grid.cells.size())});
    return writeGrid(grid, filePath);
}

bool VtkExporter::writeGrid(const VtkGrid& grid, const std::string& filePath) const
{
    std::ofstream os;
    os.exceptions(std::ios::failbit | std::ios::badbit);
    os.open(filePath);

    os << "<?xml version=\"1.0\"?>\n";
    os << "<VTKFile type=\"UnstructuredGrid\" version=\"1.0\" byte_order=\"LittleEndian\">\n";
    os << "  <UnstructuredGrid>\n";
    os << "    <Piece NumberOfPoints=\"" << grid.points.size() << "\" NumberOfCells=\"" << grid.cells.size()
       << "\">\n";

    os << "      <Points>\n";
    os << "        <DataArray type=\"Float64\" NumberOfComponents=\"3\" format=\"ascii\">\n";
    for (const auto& point : grid.points)
        os << "          " << point.x() << ' ' << point.y() << ' ' << point.z() << "\n";
    os << "        </DataArray>\n";
    os << "      </Points>\n";

    os << "      <PointData>\n";
    writeField(os, VtkField{"NodeID", grid.nodeIds});
    for (const auto& field : grid.pointFields)
        writeField(os, field);
    os << "      </PointData>\n";

    writeGridCells(os, grid.cells);

    os << "      <CellData>\n";
    for (const auto& field : grid.cellFields)
        writeField(os, field);
    os << "      </CellData>\n";
    os << "    </Piece>\n";

    os << "  </UnstructuredGrid>\n";
    os << "</VTKFile>\n";
    return true;
}

} // namespace Export
