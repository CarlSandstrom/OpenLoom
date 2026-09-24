#include "Meshing/Core/3D/RCDT/PhaseDiagnosticsExporter.h"

#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Core/3D/General/ElementGeometry3D.h"
#include "Meshing/Core/3D/RCDT/PointPhase.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"
#include "Meshing/Data/Base/MeshConnectivity.h"
#include "Topology/Topology3D.h"

#include "spdlog/spdlog.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <set>
#include <unordered_map>
#include <vector>

namespace Meshing
{

namespace
{

constexpr size_t INVALID_ID = SIZE_MAX;

int phaseCode(const PointPhase& phase)
{
    switch (phase.kind)
    {
    case PointPhaseKind::Exterior:
        return 0;
    case PointPhaseKind::InVolume:
        return 1;
    case PointPhaseKind::Ambiguous:
        return 2;
    }
    return 2;
}

/// Distance from a point to the nearest CAD surface. Unsigned: the side is
/// the phase field's job, and asking for a sign here would mean re-deciding
/// the very question the phase is there to answer.
double distanceToNearestSurface(const Point3D& point,
                                const std::vector<std::string>& surfaceIds,
                                const Geometry3D::GeometryCollection3D& geometry)
{
    double nearest = std::numeric_limits<double>::max();
    for (const auto& surfaceId : surfaceIds)
    {
        const Geometry3D::ISurface3D* surface = geometry.getSurface(surfaceId);
        if (surface)
            nearest = std::min(nearest, surface->getGap(point));
    }
    return nearest == std::numeric_limits<double>::max() ? -1.0 : nearest;
}

/// Nodes in a stable order, with the lookup from node id to the index the VTU
/// point arrays use.
struct PointBlock
{
    std::vector<size_t> nodeIds;
    std::unordered_map<size_t, size_t> indexByNodeId;
};

PointBlock buildPointBlock(const MeshData3D& meshData)
{
    PointBlock block;
    block.nodeIds.reserve(meshData.getNodes().size());
    for (const auto& [nodeId, node] : meshData.getNodes())
        block.nodeIds.push_back(nodeId);
    std::sort(block.nodeIds.begin(), block.nodeIds.end());

    for (size_t index = 0; index < block.nodeIds.size(); ++index)
        block.indexByNodeId.emplace(block.nodeIds[index], index);
    return block;
}

void writeHeader(std::ostream& os, size_t pointCount, size_t cellCount)
{
    os << "<?xml version=\"1.0\"?>\n"
       << "<VTKFile type=\"UnstructuredGrid\" version=\"1.0\" byte_order=\"LittleEndian\">\n"
       << "  <UnstructuredGrid>\n"
       << "    <Piece NumberOfPoints=\"" << pointCount << "\" NumberOfCells=\"" << cellCount << "\">\n";
}

void writeFooter(std::ostream& os)
{
    os << "    </Piece>\n  </UnstructuredGrid>\n</VTKFile>\n";
}

void writePoints(std::ostream& os, const PointBlock& block, const MeshData3D& meshData)
{
    os << "      <Points>\n"
       << "        <DataArray type=\"Float64\" NumberOfComponents=\"3\" format=\"ascii\">\n";
    for (const size_t nodeId : block.nodeIds)
    {
        const Point3D& p = meshData.getNode(nodeId)->getCoordinates();
        os << "          " << p.x() << " " << p.y() << " " << p.z() << "\n";
    }
    os << "        </DataArray>\n      </Points>\n";
}

template <typename T>
void writeArray(std::ostream& os, const std::string& type, const std::string& name, const std::vector<T>& values)
{
    os << "        <DataArray type=\"" << type << "\" Name=\"" << name << "\" format=\"ascii\">\n          ";
    for (size_t i = 0; i < values.size(); ++i)
        os << values[i] << (i + 1 == values.size() ? "\n" : " ");
    if (values.empty())
        os << "\n";
    os << "        </DataArray>\n";
}

/// connectivity/offsets/types for cells of a single VTK type.
void writeCells(std::ostream& os,
                const std::vector<std::vector<size_t>>& cells,
                int vtkCellType)
{
    os << "      <Cells>\n";

    std::vector<size_t> connectivity;
    std::vector<size_t> offsets;
    size_t running = 0;
    for (const auto& cell : cells)
    {
        connectivity.insert(connectivity.end(), cell.begin(), cell.end());
        running += cell.size();
        offsets.push_back(running);
    }
    writeArray(os, "Int64", "connectivity", connectivity);
    writeArray(os, "Int64", "offsets", offsets);
    writeArray(os, "UInt8", "types", std::vector<int>(cells.size(), vtkCellType));

    os << "      </Cells>\n";
}

} // namespace

void PhaseDiagnosticsExporter::write(const MeshData3D& meshData,
                                     const MeshConnectivity& connectivity,
                                     const Geometry3D::GeometryCollection3D& geometry,
                                     const Topology3D::Topology3D& topology,
                                     const RestrictedFaceMap& restrictedFaces,
                                     const std::string& filePrefix)
{
    const std::vector<std::string> volumeIds = topology.getAllVolumeIds();
    const std::vector<std::string> surfaceIds = topology.getAllSurfaceIds();
    const PointBlock points = buildPointBlock(meshData);
    const ElementGeometry3D elementGeometry(meshData);
    const auto& boundingNodeIds = meshData.getBoundingNodeIds();

    // Phase per tetrahedron, computed once and reused by both files. Keyed by
    // element id: nothing is mutated between here and the end of the export,
    // so ids are stable for its duration.
    std::map<size_t, PointPhase> phaseByElement;
    std::vector<std::vector<size_t>> tetCells;
    std::vector<int> tetPhase;
    std::vector<int> tetIsSupertet;
    std::vector<double> tetDistance;

    for (const auto& [elementId, element] : meshData.getElements())
    {
        const auto* tet = dynamic_cast<const TetrahedralElement*>(element.get());
        if (!tet)
            continue;

        const Point3D centroid = elementGeometry.computeCentroid(*tet);
        const PointPhase phase = classifyPointPhase(centroid, volumeIds, geometry);
        phaseByElement.emplace(elementId, phase);

        std::vector<size_t> cell;
        for (const size_t nodeId : tet->getNodeIds())
            cell.push_back(points.indexByNodeId.at(nodeId));
        tetCells.push_back(std::move(cell));

        tetPhase.push_back(phaseCode(phase));
        tetDistance.push_back(distanceToNearestSurface(centroid, surfaceIds, geometry));

        int isSupertet = 0;
        if (boundingNodeIds)
        {
            isSupertet = std::any_of(boundingNodeIds->begin(), boundingNodeIds->end(),
                                     [tet](size_t nodeId)
                                     { return tet->hasNode(nodeId); });
        }
        tetIsSupertet.push_back(isSupertet);
    }

    std::vector<double> nodeDistance;
    std::vector<long long> nodeIdArray;
    nodeDistance.reserve(points.nodeIds.size());
    for (const size_t nodeId : points.nodeIds)
    {
        nodeDistance.push_back(
            distanceToNearestSurface(meshData.getNode(nodeId)->getCoordinates(), surfaceIds, geometry));
        nodeIdArray.push_back(static_cast<long long>(nodeId));
    }

    const std::string tetPath = filePrefix + "_phase_tets.vtu";
    std::ofstream tetFile(tetPath);
    if (!tetFile)
    {
        spdlog::warn("PhaseDiagnosticsExporter: could not open {}", tetPath);
        return;
    }
    writeHeader(tetFile, points.nodeIds.size(), tetCells.size());
    writePoints(tetFile, points, meshData);
    tetFile << "      <PointData>\n";
    writeArray(tetFile, "Float64", "NodeDistanceToSurface", nodeDistance);
    writeArray(tetFile, "Int64", "NodeID", nodeIdArray);
    tetFile << "      </PointData>\n";
    writeCells(tetFile, tetCells, 10 /* VTK_TETRA */);
    tetFile << "      <CellData>\n";
    writeArray(tetFile, "Int32", "Phase", tetPhase);
    writeArray(tetFile, "Float64", "CentroidDistanceToSurface", tetDistance);
    writeArray(tetFile, "Int32", "IsSupertet", tetIsSupertet);
    tetFile << "      </CellData>\n";
    writeFooter(tetFile);

    // The face file: every face either answer calls restricted. The centroid
    // rule is exactly isPhaseBoundaryFace's condition -- both adjacent
    // tetrahedra definitive and on different sides -- with no uniqueness
    // guard and no fallback, so the disagreement shows precisely what the
    // compensation layer is buying.
    std::map<FaceKey, std::pair<bool, bool>> faceVerdicts; // face -> (live, centroid)
    for (const auto& [face, surfaceId] : restrictedFaces)
        faceVerdicts[face].first = true;

    // A set, not a counter: every face is reached once from each of its
    // two adjacent tetrahedra, so incrementing per visit would report
    // twice the number of faces actually affected.
    std::set<FaceKey> ambiguousBlockedFaces;
    for (const auto& [elementId, element] : meshData.getElements())
    {
        const auto* tet = dynamic_cast<const TetrahedralElement*>(element.get());
        if (!tet)
            continue;

        for (const auto& faceArray : tet->getFaces())
        {
            const FaceKey face(faceArray);
            const auto& [first, second] = connectivity.getFaceElements(face);
            if (first == INVALID_ID || second == INVALID_ID)
                continue;

            const auto phaseA = phaseByElement.find(first);
            const auto phaseB = phaseByElement.find(second);
            if (phaseA == phaseByElement.end() || phaseB == phaseByElement.end())
                continue;

            const PointPhase& a = phaseA->second;
            const PointPhase& b = phaseB->second;
            if (a.kind == PointPhaseKind::Ambiguous || b.kind == PointPhaseKind::Ambiguous)
            {
                if (faceVerdicts.count(face))
                    ambiguousBlockedFaces.insert(face);
                continue;
            }

            const bool differentSides =
                a.kind != b.kind || (a.kind == PointPhaseKind::InVolume && a.volumeId != b.volumeId);
            if (differentSides)
                faceVerdicts[face].second = true;
        }
    }

    std::vector<std::vector<size_t>> faceCells;
    std::vector<int> faceDiff;
    std::vector<int> facePhaseA;
    std::vector<int> facePhaseB;
    std::vector<int> faceTouchesSupertet;
    std::vector<double> faceMaxAdjacentVolume;
    size_t bothCount = 0;
    size_t liveOnlyCount = 0;
    size_t centroidOnlyCount = 0;

    for (const auto& [face, verdict] : faceVerdicts)
    {
        std::vector<size_t> cell;
        for (const size_t nodeId : face.nodeIds)
        {
            const auto found = points.indexByNodeId.find(nodeId);
            if (found != points.indexByNodeId.end())
                cell.push_back(found->second);
        }
        if (cell.size() != 3)
            continue;
        faceCells.push_back(std::move(cell));

        const auto& [live, centroid] = verdict;
        const int code = live && centroid ? 0 : (live ? 1 : 2);
        faceDiff.push_back(code);
        if (code == 0)
            ++bothCount;
        else if (code == 1)
            ++liveOnlyCount;
        else
            ++centroidOnlyCount;

        const auto& [first, second] = connectivity.getFaceElements(face);
        const auto phaseA = phaseByElement.find(first);
        const auto phaseB = phaseByElement.find(second);
        facePhaseA.push_back(phaseA == phaseByElement.end() ? -1 : phaseCode(phaseA->second));
        facePhaseB.push_back(phaseB == phaseByElement.end() ? -1 : phaseCode(phaseB->second));

        // A centroid always lies inside its own tetrahedron, but that only
        // makes it representative while the tetrahedron is small relative to
        // the geometry. An ambient tetrahedron large enough to span the model
        // can have its centroid inside the solid while the element itself is
        // mostly outside -- so element size is recorded beside the phase.
        int touchesSupertet = 0;
        double maxVolume = 0.0;
        for (const size_t elementId : {first, second})
        {
            const auto* neighbour =
                dynamic_cast<const TetrahedralElement*>(meshData.getElement(elementId));
            if (!neighbour)
                continue;
            maxVolume = std::max(maxVolume, std::abs(elementGeometry.computeVolume(*neighbour)));
            if (boundingNodeIds &&
                std::any_of(boundingNodeIds->begin(), boundingNodeIds->end(),
                            [neighbour](size_t nodeId)
                            { return neighbour->hasNode(nodeId); }))
            {
                touchesSupertet = 1;
            }
        }
        faceTouchesSupertet.push_back(touchesSupertet);
        faceMaxAdjacentVolume.push_back(maxVolume);
    }

    const std::string facePath = filePrefix + "_phase_faces.vtu";
    std::ofstream faceFile(facePath);
    if (!faceFile)
    {
        spdlog::warn("PhaseDiagnosticsExporter: could not open {}", facePath);
        return;
    }
    writeHeader(faceFile, points.nodeIds.size(), faceCells.size());
    writePoints(faceFile, points, meshData);
    faceFile << "      <PointData>\n";
    writeArray(faceFile, "Float64", "NodeDistanceToSurface", nodeDistance);
    writeArray(faceFile, "Int64", "NodeID", nodeIdArray);
    faceFile << "      </PointData>\n";
    writeCells(faceFile, faceCells, 5 /* VTK_TRIANGLE */);
    faceFile << "      <CellData>\n";
    writeArray(faceFile, "Int32", "Diff", faceDiff);
    writeArray(faceFile, "Int32", "PhaseA", facePhaseA);
    writeArray(faceFile, "Int32", "PhaseB", facePhaseB);
    writeArray(faceFile, "Int32", "TouchesSupertet", faceTouchesSupertet);
    writeArray(faceFile, "Float64", "MaxAdjacentTetVolume", faceMaxAdjacentVolume);
    faceFile << "      </CellData>\n";
    writeFooter(faceFile);

    const size_t ambiguousTets =
        static_cast<size_t>(std::count(tetPhase.begin(), tetPhase.end(), 2));
    spdlog::info("PhaseDiagnosticsExporter: {} tetrahedra, {} ambiguous ({:.1f}%)",
                 tetPhase.size(), ambiguousTets,
                 tetPhase.empty() ? 0.0 : 100.0 * static_cast<double>(ambiguousTets) / static_cast<double>(tetPhase.size()));
    spdlog::info("PhaseDiagnosticsExporter: faces -- {} agreed, {} live only, {} centroid only; "
                 "{} live faces blocked by an ambiguous centroid",
                 bothCount, liveOnlyCount, centroidOnlyCount, ambiguousBlockedFaces.size());
    spdlog::info("PhaseDiagnosticsExporter: wrote {} and {}", tetPath, facePath);
}

} // namespace Meshing
