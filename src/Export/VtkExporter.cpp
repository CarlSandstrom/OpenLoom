#include "VtkExporter.h"

#include "Meshing/Data/CurveSegmentManager.h"
#include "Meshing/Core/3D/General/DiscretizationResult3D.h"
#include "Meshing/Core/3D/General/GeometryStructures3D.h"
#include "Meshing/Data/3D/SurfaceMesh3D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/Node2D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Meshing/Data/3D/Node3D.h"
#include "Meshing/Data/Base/IElement.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace Export
{

namespace
{
// VTK cell type codes
constexpr int VTK_TETRA = 10;
constexpr int VTK_HEXAHEDRON = 12; // not used yet
constexpr int VTK_WEDGE = 13;      // prism
constexpr int VTK_PYRAMID = 14;
constexpr int VTK_LINE = 3;
constexpr int VTK_TRIANGLE = 5;
constexpr int VTK_QUAD = 9;
} // namespace

bool VtkExporter::exportMesh(const Meshing::MeshData3D& mesh, const std::string& filePath) const
{
    std::ofstream os;
    os.exceptions(std::ios::failbit | std::ios::badbit);
    os.open(filePath);

    std::vector<std::size_t> nodeIds;
    std::vector<std::size_t> elementIds;
    writeHeader(os);
    writePoints(os, mesh, nodeIds);
    writePointData(os, nodeIds);
    writeCells(os, mesh, elementIds);
    writeCellData(os, elementIds);
    writeFooter(os);
    return true;
}

bool VtkExporter::exportMesh(const Meshing::MeshData2D& mesh, const std::string& filePath) const
{
    std::ofstream os;
    os.exceptions(std::ios::failbit | std::ios::badbit);
    os.open(filePath);

    std::vector<std::size_t> nodeIds;
    std::vector<std::size_t> elementIds;
    std::size_t constraintCount = 0;

    auto domainIds = computeDomainIds(mesh);

    writeHeader(os);
    writePoints2D(os, mesh, nodeIds);
    writePointData(os, nodeIds);
    writeCells2D(os, mesh, nodeIds, elementIds, constraintCount);
    writeCellData2D(os, elementIds, constraintCount, domainIds);
    writeFooter(os);
    return true;
}

void VtkExporter::writeHeader(std::ostream& os) const
{
    os << "<?xml version=\"1.0\"?>\n";
    os << "<VTKFile type=\"UnstructuredGrid\" version=\"1.0\" byte_order=\"LittleEndian\">\n";
    os << "  <UnstructuredGrid>\n";
}

void VtkExporter::writeFooter(std::ostream& os) const
{
    os << "  </UnstructuredGrid>\n";
    os << "</VTKFile>\n";
}

void VtkExporter::writePoints(std::ostream& os, const Meshing::MeshData3D& mesh,
                              std::vector<std::size_t>& outNodeIds) const
{
    // For a deterministic index mapping, sort node IDs
    outNodeIds.clear();
    outNodeIds.reserve(mesh.getNodes().size());
    for (const auto& kv : mesh.getNodes())
    {
        outNodeIds.push_back(kv.first);
    }
    std::sort(outNodeIds.begin(), outNodeIds.end());

    os << "    <Piece NumberOfPoints=\"" << outNodeIds.size() << "\" NumberOfCells=\"" << mesh.getElements().size() << "\">\n";

    os << "      <Points>\n";
    os << "        <DataArray type=\"Float64\" NumberOfComponents=\"3\" format=\"ascii\">\n";

    for (std::size_t id : outNodeIds)
    {
        const auto* node = mesh.getNode(id);
        const auto& p = node->getCoordinates();
        os << "          " << p[0] << ' ' << p[1] << ' ' << p[2] << "\n";
    }

    os << "        </DataArray>\n";
    os << "      </Points>\n";
}

void VtkExporter::writePointData(std::ostream& os, const std::vector<std::size_t>& nodeIds) const
{
    os << "      <PointData>\n";
    os << "        <DataArray type=\"Int64\" Name=\"NodeID\" format=\"ascii\">\n          ";
    for (std::size_t i = 0; i < nodeIds.size(); ++i)
    {
        os << nodeIds[i] << (i + 1 == nodeIds.size() ? "\n" : " ");
    }
    os << "        </DataArray>\n";
    os << "      </PointData>\n";
}

static std::vector<std::size_t> sortedElementIds(const Meshing::MeshData3D& mesh)
{
    std::vector<std::size_t> ids;
    ids.reserve(mesh.getElements().size());
    for (const auto& kv : mesh.getElements())
    {
        ids.push_back(kv.first);
    }
    std::sort(ids.begin(), ids.end());
    return ids;
}

void VtkExporter::writeCells(std::ostream& os, const Meshing::MeshData3D& mesh,
                             std::vector<std::size_t>& outElementIds) const
{
    // Build node ID -> contiguous index mapping (sorted by ID to match points order)
    std::vector<std::size_t> nodeIds;
    nodeIds.reserve(mesh.getNodes().size());
    for (const auto& kv : mesh.getNodes())
        nodeIds.push_back(kv.first);
    std::sort(nodeIds.begin(), nodeIds.end());

    std::unordered_map<std::size_t, std::size_t> nodeIndex;
    nodeIndex.reserve(nodeIds.size());
    for (std::size_t i = 0; i < nodeIds.size(); ++i)
        nodeIndex[nodeIds[i]] = i;

    // Prepare element ids
    const auto elemIds = sortedElementIds(mesh);

    // Connectivity and offsets arrays (VTU style)
    std::vector<unsigned int> connectivity;
    std::vector<unsigned int> offsets;
    std::vector<unsigned char> types;

    connectivity.reserve(elemIds.size() * 4); // rough reserve for tets
    offsets.reserve(elemIds.size());
    types.reserve(elemIds.size());
    outElementIds.clear();
    outElementIds.reserve(elemIds.size());

    unsigned int runningOffset = 0;

    for (std::size_t eid : elemIds)
    {
        const auto* e = mesh.getElement(eid);
        const int vtkType = vtkCellTypeFor(*e);
        if (vtkType < 0)
        {
            // Skip unsupported element types
            continue;
        }
        const auto& nodes = e->getNodeIds();
        for (std::size_t nid : nodes)
        {
            connectivity.push_back(static_cast<unsigned int>(nodeIndex.at(nid)));
        }
        runningOffset += static_cast<unsigned int>(nodes.size());
        offsets.push_back(runningOffset);
        types.push_back(static_cast<unsigned char>(vtkType));
        outElementIds.push_back(eid);
    }

    os << "      <Cells>\n";

    os << "        <DataArray type=\"Int32\" Name=\"connectivity\" format=\"ascii\">\n          ";
    for (std::size_t i = 0; i < connectivity.size(); ++i)
    {
        os << connectivity[i] << (i + 1 == connectivity.size() ? "\n" : " ");
    }
    os << "        </DataArray>\n";

    os << "        <DataArray type=\"Int32\" Name=\"offsets\" format=\"ascii\">\n          ";
    for (std::size_t i = 0; i < offsets.size(); ++i)
    {
        os << offsets[i] << (i + 1 == offsets.size() ? "\n" : " ");
    }
    os << "        </DataArray>\n";

    os << "        <DataArray type=\"UInt8\" Name=\"types\" format=\"ascii\">\n          ";
    for (std::size_t i = 0; i < types.size(); ++i)
    {
        os << static_cast<unsigned int>(types[i]) << (i + 1 == types.size() ? "\n" : " ");
    }
    os << "        </DataArray>\n";

    os << "      </Cells>\n";
}

void VtkExporter::writeCellData(std::ostream& os, const std::vector<std::size_t>& elementIds) const
{
    os << "      <CellData>\n";
    os << "        <DataArray type=\"Int64\" Name=\"ElementID\" format=\"ascii\">\n          ";
    for (std::size_t i = 0; i < elementIds.size(); ++i)
    {
        os << elementIds[i] << (i + 1 == elementIds.size() ? "\n" : " ");
    }
    os << "        </DataArray>\n";
    os << "      </CellData>\n";

    // Close Piece tag started in writePoints
    os << "    </Piece>\n";
}

void VtkExporter::writePoints2D(std::ostream& os, const Meshing::MeshData2D& mesh,
                                std::vector<std::size_t>& outNodeIds) const
{
    outNodeIds.clear();
    outNodeIds.reserve(mesh.getNodes().size());
    for (const auto& kv : mesh.getNodes())
    {
        outNodeIds.push_back(kv.first);
    }
    std::sort(outNodeIds.begin(), outNodeIds.end());

    const std::size_t totalCells = mesh.getElements().size() + mesh.getCurveSegmentManager().size();
    os << "    <Piece NumberOfPoints=\"" << outNodeIds.size()
       << "\" NumberOfCells=\"" << totalCells << "\">\n";

    os << "      <Points>\n";
    os << "        <DataArray type=\"Float64\" NumberOfComponents=\"3\" format=\"ascii\">\n";

    for (std::size_t id : outNodeIds)
    {
        const auto* node = mesh.getNode(id);
        const auto& p = node->getCoordinates();
        os << "          " << p[0] << ' ' << p[1] << " 0\n";
    }

    os << "        </DataArray>\n";
    os << "      </Points>\n";
}

void VtkExporter::writeCells2D(std::ostream& os, const Meshing::MeshData2D& mesh,
                               const std::vector<std::size_t>& nodeIds,
                               std::vector<std::size_t>& outElementIds,
                               std::size_t& outConstraintCount) const
{
    // Build node ID -> contiguous index mapping
    std::unordered_map<std::size_t, std::size_t> nodeIndex;
    nodeIndex.reserve(nodeIds.size());
    for (std::size_t i = 0; i < nodeIds.size(); ++i)
        nodeIndex[nodeIds[i]] = i;

    // Sorted element IDs
    std::vector<std::size_t> elemIds;
    elemIds.reserve(mesh.getElements().size());
    for (const auto& kv : mesh.getElements())
        elemIds.push_back(kv.first);
    std::sort(elemIds.begin(), elemIds.end());

    std::vector<unsigned int> connectivity;
    std::vector<unsigned int> offsets;
    std::vector<unsigned char> types;

    const auto& curveSegmentManager = mesh.getCurveSegmentManager();
    const std::size_t totalCells = elemIds.size() + curveSegmentManager.size();
    connectivity.reserve(elemIds.size() * 3 + curveSegmentManager.size() * 2);
    offsets.reserve(totalCells);
    types.reserve(totalCells);
    outElementIds.clear();
    outElementIds.reserve(elemIds.size());

    unsigned int runningOffset = 0;

    // Write element cells first
    for (std::size_t eid : elemIds)
    {
        const auto* e = mesh.getElement(eid);
        const int vtkType = vtkCellTypeFor(*e);
        if (vtkType < 0)
            continue;
        const auto& nodes = e->getNodeIds();
        for (std::size_t nid : nodes)
        {
            connectivity.push_back(static_cast<unsigned int>(nodeIndex.at(nid)));
        }
        runningOffset += static_cast<unsigned int>(nodes.size());
        offsets.push_back(runningOffset);
        types.push_back(static_cast<unsigned char>(vtkType));
        outElementIds.push_back(eid);
    }

    // Write constraint segments as VTK_LINE cells
    outConstraintCount = curveSegmentManager.size();
    for (const auto& [segId, seg] : curveSegmentManager.getAllSegments())
    {
        connectivity.push_back(static_cast<unsigned int>(nodeIndex.at(seg.nodeId1)));
        connectivity.push_back(static_cast<unsigned int>(nodeIndex.at(seg.nodeId2)));
        runningOffset += 2;
        offsets.push_back(runningOffset);
        types.push_back(static_cast<unsigned char>(VTK_LINE));
    }

    os << "      <Cells>\n";

    os << "        <DataArray type=\"Int32\" Name=\"connectivity\" format=\"ascii\">\n          ";
    for (std::size_t i = 0; i < connectivity.size(); ++i)
    {
        os << connectivity[i] << (i + 1 == connectivity.size() ? "\n" : " ");
    }
    os << "        </DataArray>\n";

    os << "        <DataArray type=\"Int32\" Name=\"offsets\" format=\"ascii\">\n          ";
    for (std::size_t i = 0; i < offsets.size(); ++i)
    {
        os << offsets[i] << (i + 1 == offsets.size() ? "\n" : " ");
    }
    os << "        </DataArray>\n";

    os << "        <DataArray type=\"UInt8\" Name=\"types\" format=\"ascii\">\n          ";
    for (std::size_t i = 0; i < types.size(); ++i)
    {
        os << static_cast<unsigned int>(types[i]) << (i + 1 == types.size() ? "\n" : " ");
    }
    os << "        </DataArray>\n";

    os << "      </Cells>\n";
}

void VtkExporter::writeCellData2D(std::ostream& os, const std::vector<std::size_t>& elementIds,
                                  std::size_t constraintCount,
                                  const std::unordered_map<std::size_t, int>& domainIds) const
{
    const std::size_t totalCells = elementIds.size() + constraintCount;

    os << "      <CellData>\n";

    // ElementID array (constraints get ID = 0)
    os << "        <DataArray type=\"Int64\" Name=\"ElementID\" format=\"ascii\">\n          ";
    for (std::size_t i = 0; i < totalCells; ++i)
    {
        if (i < elementIds.size())
            os << elementIds[i];
        else
            os << 0;
        os << (i + 1 == totalCells ? "\n" : " ");
    }
    os << "        </DataArray>\n";

    // EdgeRole array: 0 = element, 1 = constraint edge
    os << "        <DataArray type=\"Int32\" Name=\"EdgeRole\" format=\"ascii\">\n          ";
    for (std::size_t i = 0; i < totalCells; ++i)
    {
        os << (i < elementIds.size() ? 0 : 1);
        os << (i + 1 == totalCells ? "\n" : " ");
    }
    os << "        </DataArray>\n";

    // DomainID array: only written when domain classification produced results
    if (!domainIds.empty())
    {
        os << "        <DataArray type=\"Int32\" Name=\"DomainID\" format=\"ascii\">\n          ";
        for (std::size_t i = 0; i < totalCells; ++i)
        {
            if (i < elementIds.size())
            {
                auto it = domainIds.find(elementIds[i]);
                os << (it != domainIds.end() ? it->second : -1);
            }
            else
            {
                os << -1; // constraint edges get domain -1
            }
            os << (i + 1 == totalCells ? "\n" : " ");
        }
        os << "        </DataArray>\n";
    }

    os << "      </CellData>\n";
    os << "    </Piece>\n";
}

std::unordered_map<std::size_t, int> VtkExporter::computeDomainIds(const Meshing::MeshData2D& mesh)
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

int VtkExporter::vtkCellTypeFor(const Meshing::IElement& element)
{
    using Meshing::ElementType;
    switch (element.getType())
    {
    case ElementType::TETRAHEDRON:
        return VTK_TETRA;
    case ElementType::HEXAHEDRON:
        return VTK_HEXAHEDRON;
    case ElementType::PRISM:
        return VTK_WEDGE;
    case ElementType::PYRAMID:
        return VTK_PYRAMID;
    case ElementType::TRIANGLE:
        return VTK_TRIANGLE;
    case ElementType::QUADRILATERAL:
        return VTK_QUAD;
    default:
        return -1;
    }
}

Meshing::MeshData3D VtkExporter::convertToMeshData3D(const Meshing::MeshData2D& mesh2D)
{
    // Simply use the MeshData3D constructor that handles the conversion
    return Meshing::MeshData3D(mesh2D);
}

bool VtkExporter::writeEdgeMesh(const Meshing::DiscretizationResult3D& result,
                                const std::string& filePath) const
{
    // Count line segments across all topology edges
    std::size_t totalLines = 0;
    for (const auto& [edgeId, pointIndices] : result.edgeIdToPointIndicesMap)
    {
        if (pointIndices.size() >= 2)
            totalLines += pointIndices.size() - 1;
    }

    std::ofstream os;
    os.exceptions(std::ios::failbit | std::ios::badbit);
    os.open(filePath);

    writeHeader(os);

    const std::size_t numPoints = result.points.size();
    os << "    <Piece NumberOfPoints=\"" << numPoints << "\" NumberOfCells=\"" << totalLines << "\">\n";

    // Points: all sampled points in discretization order (index == point ID)
    os << "      <Points>\n";
    os << "        <DataArray type=\"Float64\" NumberOfComponents=\"3\" format=\"ascii\">\n";
    for (const auto& p : result.points)
    {
        os << "          " << p[0] << ' ' << p[1] << ' ' << p[2] << "\n";
    }
    os << "        </DataArray>\n";
    os << "      </Points>\n";

    // PointData: global point index as NodeID
    os << "      <PointData>\n";
    os << "        <DataArray type=\"Int64\" Name=\"NodeID\" format=\"ascii\">\n          ";
    for (std::size_t i = 0; i < numPoints; ++i)
    {
        os << i << (i + 1 == numPoints ? "\n" : " ");
    }
    os << "        </DataArray>\n";
    os << "      </PointData>\n";

    // Cells: one VTK_LINE per consecutive point pair within each topology edge
    std::vector<unsigned int> connectivity;
    std::vector<unsigned int> offsets;
    std::vector<int> edgeIndices; // per-cell EdgeID for coloring

    connectivity.reserve(totalLines * 2);
    offsets.reserve(totalLines);
    edgeIndices.reserve(totalLines);

    unsigned int runningOffset = 0;
    int edgeIndex = 0;

    for (const auto& [edgeId, pointIndices] : result.edgeIdToPointIndicesMap)
    {
        for (std::size_t i = 0; i + 1 < pointIndices.size(); ++i)
        {
            connectivity.push_back(static_cast<unsigned int>(pointIndices[i]));
            connectivity.push_back(static_cast<unsigned int>(pointIndices[i + 1]));
            runningOffset += 2;
            offsets.push_back(runningOffset);
            edgeIndices.push_back(edgeIndex);
        }
        ++edgeIndex;
    }

    os << "      <Cells>\n";

    os << "        <DataArray type=\"Int32\" Name=\"connectivity\" format=\"ascii\">\n          ";
    for (std::size_t i = 0; i < connectivity.size(); ++i)
    {
        os << connectivity[i] << (i + 1 == connectivity.size() ? "\n" : " ");
    }
    os << "        </DataArray>\n";

    os << "        <DataArray type=\"Int32\" Name=\"offsets\" format=\"ascii\">\n          ";
    for (std::size_t i = 0; i < offsets.size(); ++i)
    {
        os << offsets[i] << (i + 1 == offsets.size() ? "\n" : " ");
    }
    os << "        </DataArray>\n";

    os << "        <DataArray type=\"UInt8\" Name=\"types\" format=\"ascii\">\n          ";
    for (std::size_t i = 0; i < offsets.size(); ++i)
    {
        os << static_cast<unsigned int>(VTK_LINE) << (i + 1 == offsets.size() ? "\n" : " ");
    }
    os << "        </DataArray>\n";

    os << "      </Cells>\n";

    // CellData: EdgeID for color-by-edge inspection in ParaView
    os << "      <CellData>\n";
    os << "        <DataArray type=\"Int32\" Name=\"EdgeID\" format=\"ascii\">\n          ";
    for (std::size_t i = 0; i < edgeIndices.size(); ++i)
    {
        os << edgeIndices[i] << (i + 1 == edgeIndices.size() ? "\n" : " ");
    }
    os << "        </DataArray>\n";
    os << "      </CellData>\n";
    os << "    </Piece>\n";

    writeFooter(os);
    return true;
}

bool VtkExporter::writeSurfaceMesh(const Meshing::DiscretizationResult3D& disc3D,
                                   const std::vector<Meshing::ConstrainedSubfacet3D>& subfacets,
                                   const std::string& filePath) const
{
    // Build a stable integer mapping: surfaceId string → 0-based index (sorted for stability)
    std::vector<std::string> sortedSurfaceIds;
    for (const auto& subfacet : subfacets)
    {
        sortedSurfaceIds.push_back(subfacet.geometryId);
    }
    std::sort(sortedSurfaceIds.begin(), sortedSurfaceIds.end());
    sortedSurfaceIds.erase(std::unique(sortedSurfaceIds.begin(), sortedSurfaceIds.end()),
                           sortedSurfaceIds.end());

    std::unordered_map<std::string, int> surfaceIdToIndex;
    for (int i = 0; i < static_cast<int>(sortedSurfaceIds.size()); ++i)
    {
        surfaceIdToIndex[sortedSurfaceIds[i]] = i;
    }

    const std::size_t numPoints = disc3D.points.size();
    const std::size_t numCells = subfacets.size();

    std::ofstream os;
    os.exceptions(std::ios::failbit | std::ios::badbit);
    os.open(filePath);

    writeHeader(os);

    os << "    <Piece NumberOfPoints=\"" << numPoints << "\" NumberOfCells=\"" << numCells << "\">\n";

    // Points: all discretization points in index order (index == node ID in surface-mesher path)
    os << "      <Points>\n";
    os << "        <DataArray type=\"Float64\" NumberOfComponents=\"3\" format=\"ascii\">\n";
    for (const auto& p : disc3D.points)
    {
        os << "          " << p[0] << ' ' << p[1] << ' ' << p[2] << "\n";
    }
    os << "        </DataArray>\n";
    os << "      </Points>\n";

    // PointData: global point index as NodeID
    os << "      <PointData>\n";
    os << "        <DataArray type=\"Int64\" Name=\"NodeID\" format=\"ascii\">\n          ";
    for (std::size_t i = 0; i < numPoints; ++i)
    {
        os << i << (i + 1 == numPoints ? "\n" : " ");
    }
    os << "        </DataArray>\n";
    os << "      </PointData>\n";

    // Cells: one VTK_TRIANGLE per subfacet
    std::vector<unsigned int> connectivity;
    std::vector<unsigned int> offsets;
    std::vector<int> surfaceIndices;

    connectivity.reserve(numCells * 3);
    offsets.reserve(numCells);
    surfaceIndices.reserve(numCells);

    unsigned int runningOffset = 0;
    for (const auto& subfacet : subfacets)
    {
        connectivity.push_back(static_cast<unsigned int>(subfacet.nodeId1));
        connectivity.push_back(static_cast<unsigned int>(subfacet.nodeId2));
        connectivity.push_back(static_cast<unsigned int>(subfacet.nodeId3));
        runningOffset += 3;
        offsets.push_back(runningOffset);
        surfaceIndices.push_back(surfaceIdToIndex.at(subfacet.geometryId));
    }

    os << "      <Cells>\n";

    os << "        <DataArray type=\"Int32\" Name=\"connectivity\" format=\"ascii\">\n          ";
    for (std::size_t i = 0; i < connectivity.size(); ++i)
    {
        os << connectivity[i] << (i + 1 == connectivity.size() ? "\n" : " ");
    }
    os << "        </DataArray>\n";

    os << "        <DataArray type=\"Int32\" Name=\"offsets\" format=\"ascii\">\n          ";
    for (std::size_t i = 0; i < offsets.size(); ++i)
    {
        os << offsets[i] << (i + 1 == offsets.size() ? "\n" : " ");
    }
    os << "        </DataArray>\n";

    os << "        <DataArray type=\"UInt8\" Name=\"types\" format=\"ascii\">\n          ";
    for (std::size_t i = 0; i < numCells; ++i)
    {
        os << static_cast<unsigned int>(VTK_TRIANGLE) << (i + 1 == numCells ? "\n" : " ");
    }
    os << "        </DataArray>\n";

    os << "      </Cells>\n";

    // CellData: SurfaceID and ConstraintRole per subfacet
    os << "      <CellData>\n";
    os << "        <DataArray type=\"Int32\" Name=\"SurfaceID\" format=\"ascii\">\n          ";
    for (std::size_t i = 0; i < surfaceIndices.size(); ++i)
    {
        os << surfaceIndices[i] << (i + 1 == surfaceIndices.size() ? "\n" : " ");
    }
    os << "        </DataArray>\n";
    os << "        <DataArray type=\"Int32\" Name=\"ConstraintRole\" format=\"ascii\">\n          ";
    for (std::size_t i = 0; i < numCells; ++i)
    {
        int value = subfacets[i].role == Meshing::ConstraintRole::Boundary ? 0 : 1;
        os << value << (i + 1 == numCells ? "\n" : " ");
    }
    os << "        </DataArray>\n";
    os << "      </CellData>\n";
    os << "    </Piece>\n";

    writeFooter(os);
    return true;
}

bool VtkExporter::writeSurfaceMesh(const Meshing::MeshData3D& mesh,
                                   const std::vector<Meshing::ConstrainedSubfacet3D>& subfacets,
                                   const std::string& filePath) const
{
    // Build surface ID index (same as disc3D overload)
    std::vector<std::string> sortedSurfaceIds;
    for (const auto& subfacet : subfacets)
        sortedSurfaceIds.push_back(subfacet.geometryId);
    std::sort(sortedSurfaceIds.begin(), sortedSurfaceIds.end());
    sortedSurfaceIds.erase(std::unique(sortedSurfaceIds.begin(), sortedSurfaceIds.end()),
                           sortedSurfaceIds.end());

    std::unordered_map<std::string, int> surfaceIdToIndex;
    for (int i = 0; i < static_cast<int>(sortedSurfaceIds.size()); ++i)
        surfaceIdToIndex[sortedSurfaceIds[i]] = i;

    // Collect nodes sorted by ID so VTK point index == node ID.
    std::vector<std::pair<size_t, const Meshing::Node3D*>> sortedNodes;
    for (const auto& [id, node] : mesh.getNodes())
        sortedNodes.emplace_back(id, node.get());
    std::sort(sortedNodes.begin(), sortedNodes.end(), [](const auto& a, const auto& b) {
        return a.first < b.first;
    });

    const std::size_t numPoints = sortedNodes.size();
    const std::size_t numCells = subfacets.size();

    std::ofstream os;
    os.exceptions(std::ios::failbit | std::ios::badbit);
    os.open(filePath);

    writeHeader(os);

    os << "    <Piece NumberOfPoints=\"" << numPoints << "\" NumberOfCells=\"" << numCells << "\">\n";

    os << "      <Points>\n";
    os << "        <DataArray type=\"Float64\" NumberOfComponents=\"3\" format=\"ascii\">\n";
    for (const auto& [id, node] : sortedNodes)
    {
        const auto& p = node->getCoordinates();
        os << "          " << p[0] << ' ' << p[1] << ' ' << p[2] << "\n";
    }
    os << "        </DataArray>\n";
    os << "      </Points>\n";

    os << "      <PointData>\n";
    os << "        <DataArray type=\"Int64\" Name=\"NodeID\" format=\"ascii\">\n          ";
    for (std::size_t i = 0; i < numPoints; ++i)
        os << sortedNodes[i].first << (i + 1 == numPoints ? "\n" : " ");
    os << "        </DataArray>\n";
    os << "      </PointData>\n";

    // Cells
    std::vector<unsigned int> connectivity;
    std::vector<unsigned int> offsets;
    std::vector<int> surfaceIndices;
    connectivity.reserve(numCells * 3);
    offsets.reserve(numCells);
    surfaceIndices.reserve(numCells);

    unsigned int runningOffset = 0;
    for (const auto& subfacet : subfacets)
    {
        connectivity.push_back(static_cast<unsigned int>(subfacet.nodeId1));
        connectivity.push_back(static_cast<unsigned int>(subfacet.nodeId2));
        connectivity.push_back(static_cast<unsigned int>(subfacet.nodeId3));
        runningOffset += 3;
        offsets.push_back(runningOffset);
        surfaceIndices.push_back(surfaceIdToIndex.at(subfacet.geometryId));
    }

    os << "      <Cells>\n";
    os << "        <DataArray type=\"Int32\" Name=\"connectivity\" format=\"ascii\">\n          ";
    for (std::size_t i = 0; i < connectivity.size(); ++i)
        os << connectivity[i] << (i + 1 == connectivity.size() ? "\n" : " ");
    os << "        </DataArray>\n";

    os << "        <DataArray type=\"Int32\" Name=\"offsets\" format=\"ascii\">\n          ";
    for (std::size_t i = 0; i < offsets.size(); ++i)
        os << offsets[i] << (i + 1 == offsets.size() ? "\n" : " ");
    os << "        </DataArray>\n";

    os << "        <DataArray type=\"UInt8\" Name=\"types\" format=\"ascii\">\n          ";
    for (std::size_t i = 0; i < numCells; ++i)
        os << static_cast<unsigned int>(VTK_TRIANGLE) << (i + 1 == numCells ? "\n" : " ");
    os << "        </DataArray>\n";
    os << "      </Cells>\n";

    os << "      <CellData>\n";
    os << "        <DataArray type=\"Int32\" Name=\"SurfaceID\" format=\"ascii\">\n          ";
    for (std::size_t i = 0; i < surfaceIndices.size(); ++i)
        os << surfaceIndices[i] << (i + 1 == surfaceIndices.size() ? "\n" : " ");
    os << "        </DataArray>\n";
    os << "        <DataArray type=\"Int32\" Name=\"ConstraintRole\" format=\"ascii\">\n          ";
    for (std::size_t i = 0; i < numCells; ++i)
    {
        int value = subfacets[i].role == Meshing::ConstraintRole::Boundary ? 0 : 1;
        os << value << (i + 1 == numCells ? "\n" : " ");
    }
    os << "        </DataArray>\n";
    os << "      </CellData>\n";
    os << "    </Piece>\n";

    writeFooter(os);
    return true;
}

bool VtkExporter::writeSurfaceMesh(const Meshing::SurfaceMesh3D& surfaceMesh,
                                   const std::string& filePath) const
{
    // Build a stable integer mapping: surfaceId string → 0-based index (sorted for stability).
    std::vector<std::string> sortedSurfaceIds;
    sortedSurfaceIds.reserve(surfaceMesh.faceTriangleIds.size());
    for (const auto& [surfaceId, unused] : surfaceMesh.faceTriangleIds)
        sortedSurfaceIds.push_back(surfaceId);
    std::sort(sortedSurfaceIds.begin(), sortedSurfaceIds.end());

    std::unordered_map<std::string, int> surfaceIdToIndex;
    for (int i = 0; i < static_cast<int>(sortedSurfaceIds.size()); ++i)
        surfaceIdToIndex[sortedSurfaceIds[i]] = i;

    // Build per-triangle surface index array in triangle-ID order.
    const size_t numTriangles = surfaceMesh.triangles.size();
    std::vector<int> surfaceIndices(numTriangles, -1);
    for (const auto& [surfaceId, triangleIds] : surfaceMesh.faceTriangleIds)
    {
        const int index = surfaceIdToIndex.at(surfaceId);
        for (size_t triangleId : triangleIds)
            surfaceIndices[triangleId] = index;
    }

    const size_t numPoints = surfaceMesh.nodes.size();

    std::ofstream os;
    os.exceptions(std::ios::failbit | std::ios::badbit);
    os.open(filePath);

    writeHeader(os);

    os << "    <Piece NumberOfPoints=\"" << numPoints << "\" NumberOfCells=\"" << numTriangles << "\">\n";

    os << "      <Points>\n";
    os << "        <DataArray type=\"Float64\" NumberOfComponents=\"3\" format=\"ascii\">\n";
    for (const auto& p : surfaceMesh.nodes)
        os << "          " << p[0] << ' ' << p[1] << ' ' << p[2] << "\n";
    os << "        </DataArray>\n";
    os << "      </Points>\n";

    os << "      <PointData>\n";
    os << "        <DataArray type=\"Int64\" Name=\"NodeID\" format=\"ascii\">\n          ";
    for (size_t i = 0; i < numPoints; ++i)
        os << i << (i + 1 == numPoints ? "\n" : " ");
    os << "        </DataArray>\n";
    os << "      </PointData>\n";

    os << "      <Cells>\n";

    os << "        <DataArray type=\"Int32\" Name=\"connectivity\" format=\"ascii\">\n          ";
    for (size_t i = 0; i < numTriangles; ++i)
    {
        const auto& tri = surfaceMesh.triangles[i];
        os << tri[0] << ' ' << tri[1] << ' ' << tri[2];
        os << (i + 1 == numTriangles ? "\n" : " ");
    }
    os << "        </DataArray>\n";

    os << "        <DataArray type=\"Int32\" Name=\"offsets\" format=\"ascii\">\n          ";
    for (size_t i = 0; i < numTriangles; ++i)
        os << (i + 1) * 3 << (i + 1 == numTriangles ? "\n" : " ");
    os << "        </DataArray>\n";

    os << "        <DataArray type=\"UInt8\" Name=\"types\" format=\"ascii\">\n          ";
    for (size_t i = 0; i < numTriangles; ++i)
        os << static_cast<unsigned int>(VTK_TRIANGLE) << (i + 1 == numTriangles ? "\n" : " ");
    os << "        </DataArray>\n";

    os << "      </Cells>\n";

    os << "      <CellData>\n";
    os << "        <DataArray type=\"Int32\" Name=\"SurfaceID\" format=\"ascii\">\n          ";
    for (size_t i = 0; i < surfaceIndices.size(); ++i)
        os << surfaceIndices[i] << (i + 1 == surfaceIndices.size() ? "\n" : " ");
    os << "        </DataArray>\n";
    os << "      </CellData>\n";
    os << "    </Piece>\n";

    writeFooter(os);
    return true;
}

} // namespace Export
