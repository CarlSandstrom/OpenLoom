#include "VtkExporter.h"

#include "Meshing/Data/IElement.h"
#include "Meshing/Data/MeshData3D.h"
#include "Meshing/Data/Node3D.h"

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
constexpr int VTK_TRIANGLE = 5;
constexpr int VTK_QUAD = 9;
} // namespace

bool VtkExporter::exportMesh(const Meshing::MeshData& mesh, const std::string& filePath) const
{
    std::ofstream os(filePath);
    if (!os.is_open())
    {
        return false;
    }

    writeHeader_(os);
    writePoints_(os, mesh);
    writeCells_(os, mesh);
    writeFooter_(os);
    return true;
}

void VtkExporter::writeHeader_(std::ostream& os) const
{
    os << "<?xml version=\"1.0\"?>\n";
    os << "<VTKFile type=\"UnstructuredGrid\" version=\"1.0\" byte_order=\"LittleEndian\">\n";
    os << "  <UnstructuredGrid>\n";
}

void VtkExporter::writeFooter_(std::ostream& os) const
{
    os << "  </UnstructuredGrid>\n";
    os << "</VTKFile>\n";
}

void VtkExporter::writePoints_(std::ostream& os, const Meshing::MeshData& mesh) const
{
    // For a deterministic index mapping, sort node IDs
    std::vector<std::size_t> nodeIds;
    nodeIds.reserve(mesh.getNodes().size());
    for (const auto& kv : mesh.getNodes())
    {
        nodeIds.push_back(kv.first);
    }
    std::sort(nodeIds.begin(), nodeIds.end());

    os << "    <Piece NumberOfPoints=\"" << nodeIds.size() << "\" NumberOfCells=\"" << mesh.getElements().size() << "\">\n";

    os << "      <Points>\n";
    os << "        <DataArray type=\"Float64\" NumberOfComponents=\"3\" format=\"ascii\">\n";

    for (std::size_t id : nodeIds)
    {
        const auto* node = mesh.getNode(id);
        const auto& p = node->getCoordinates();
        os << "          " << p[0] << ' ' << p[1] << ' ' << p[2] << "\n";
    }

    os << "        </DataArray>\n";
    os << "      </Points>\n";
}

static std::vector<std::size_t> sortedElementIds(const Meshing::MeshData& mesh)
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

void VtkExporter::writeCells_(std::ostream& os, const Meshing::MeshData& mesh) const
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

    unsigned int runningOffset = 0;

    for (std::size_t eid : elemIds)
    {
        const auto* e = mesh.getElement(eid);
        const int vtkType = vtkCellTypeFor_(*e);
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

    // Close Piece tag started in writePoints_
    os << "    </Piece>\n";
}

int VtkExporter::vtkCellTypeFor_(const Meshing::IElement& element)
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

} // namespace Export
