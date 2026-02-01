#include "VtkExporter.h"

#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/Node2D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Meshing/Data/3D/Node3D.h"
#include "Meshing/Data/Base/IElement.h"

#include <algorithm>
#include <fstream>
#include <unordered_map>
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

    writeHeader(os);
    writePoints2D(os, mesh, nodeIds);
    writePointData(os, nodeIds);
    writeCells2D(os, mesh, nodeIds, elementIds, constraintCount);
    writeCellData2D(os, elementIds, constraintCount);
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

    const std::size_t totalCells = mesh.getElements().size() + mesh.getConstrainedSegmentCount();
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

    const auto& constraints = mesh.getConstrainedSegments();
    const std::size_t totalCells = elemIds.size() + constraints.size();
    connectivity.reserve(elemIds.size() * 3 + constraints.size() * 2);
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
    outConstraintCount = constraints.size();
    for (const auto& seg : constraints)
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
                                  std::size_t constraintCount) const
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

    os << "      </CellData>\n";
    os << "    </Piece>\n";
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

} // namespace Export
