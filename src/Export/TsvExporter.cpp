#include "TsvExporter.h"

#include "Meshing/Core/3D/General/DiscretizationResult3D.h"
#include "Meshing/Core/3D/General/GeometryStructures3D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/Node2D.h"
#include "Meshing/Data/3D/SurfaceMesh3D.h"
#include "Meshing/Data/3D/VolumeMesh3D.h"
#include "Meshing/Data/Base/IElement.h"
#include "Meshing/Data/CurveSegmentManager.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <fstream>
#include <map>
#include <string>
#include <vector>

namespace Export
{

namespace
{

constexpr const char* NOT_APPLICABLE = "-";
constexpr const char* CELL_HEADER = "kind\tid\tnodes\tgeometry_id\trole\n";

std::ofstream openTable(const std::string& stem, const std::string& table)
{
    std::ofstream os;
    os.exceptions(std::ios::failbit | std::ios::badbit);
    os.open(stem + "." + table + ".tsv");
    return os;
}

// Shortest representation that parses back to the same double, so two
// outputs compare equal exactly when the coordinates are bit-identical.
std::string formatDouble(double value)
{
    std::array<char, 32> buffer{};
    const auto result = std::to_chars(buffer.data(), buffer.data() + buffer.size(), value);
    return std::string(buffer.data(), result.ptr);
}

template <typename Container, typename Format>
std::string joinWithCommas(const Container& values, Format format)
{
    if (values.empty())
        return NOT_APPLICABLE;

    std::string joined;
    for (const auto& value : values)
    {
        if (!joined.empty())
            joined += ',';
        joined += format(value);
    }
    return joined;
}

std::string joinNodeIds(const auto& nodeIds)
{
    return joinWithCommas(nodeIds, [](std::size_t id)
                          { return std::to_string(id); });
}

std::string formatRole(Meshing::ConstraintRole role)
{
    return role == Meshing::ConstraintRole::Boundary ? "boundary" : "interior";
}

std::string formatElementKind(Meshing::ElementType type)
{
    switch (type)
    {
    case Meshing::ElementType::TETRAHEDRON:
        return "tetrahedron";
    case Meshing::ElementType::HEXAHEDRON:
        return "hexahedron";
    case Meshing::ElementType::PRISM:
        return "prism";
    case Meshing::ElementType::PYRAMID:
        return "pyramid";
    case Meshing::ElementType::TRIANGLE:
        return "triangle";
    case Meshing::ElementType::QUADRILATERAL:
        return "quadrilateral";
    }
    return "unknown";
}

void writeCellRow(std::ostream& os, const std::string& kind, const std::string& id,
                  const std::string& nodes, const std::string& geometryId, const std::string& role)
{
    os << kind << '\t' << id << '\t' << nodes << '\t' << geometryId << '\t' << role << '\n';
}

void writePointNodes(const std::vector<Meshing::Point3D>& points, const std::string& stem)
{
    auto os = openTable(stem, "nodes");
    os << "node_id\tx\ty\tz\n";
    for (std::size_t i = 0; i < points.size(); ++i)
    {
        const auto& point = points[i];
        os << i << '\t' << formatDouble(point.x()) << '\t' << formatDouble(point.y()) << '\t'
           << formatDouble(point.z()) << '\n';
    }
}

// The triangle -> geometry id lookup inverted from a per-surface grouping.
std::vector<std::string> triangleGeometryIds(const std::map<std::string, std::vector<std::size_t>>& groups,
                                             std::size_t triangleCount)
{
    std::vector<std::string> geometryIds(triangleCount, NOT_APPLICABLE);
    for (const auto& [geometryId, triangleIds] : groups)
    {
        for (std::size_t triangleId : triangleIds)
            geometryIds[triangleId] = geometryId;
    }
    return geometryIds;
}

void writeTriangles(std::ostream& os, const std::vector<std::array<std::size_t, 3>>& triangles,
                    const std::map<std::string, std::vector<std::size_t>>& groups)
{
    const auto geometryIds = triangleGeometryIds(groups, triangles.size());
    for (std::size_t i = 0; i < triangles.size(); ++i)
        writeCellRow(os, "triangle", std::to_string(i), joinNodeIds(triangles[i]), geometryIds[i], NOT_APPLICABLE);
}

void writeNodeChains(std::ostream& os, const std::string& kind,
                     const std::map<std::string, std::vector<std::size_t>>& chains)
{
    std::size_t index = 0;
    for (const auto& [geometryId, nodeIds] : chains)
        writeCellRow(os, kind, std::to_string(index++), joinNodeIds(nodeIds), geometryId, NOT_APPLICABLE);
}

template <typename Map>
std::vector<typename Map::key_type> sortedKeys(const Map& map)
{
    std::vector<typename Map::key_type> keys;
    keys.reserve(map.size());
    for (const auto& entry : map)
        keys.push_back(entry.first);
    std::sort(keys.begin(), keys.end());
    return keys;
}

} // namespace

void TsvExporter::writeMesh(const Meshing::MeshData2D& mesh, const std::string& stem)
{
    auto nodes = openTable(stem, "nodes");
    nodes << "node_id\tx\ty\n";
    for (std::size_t nodeId : sortedKeys(mesh.getNodes()))
    {
        const auto& point = mesh.getNode(nodeId)->getCoordinates();
        nodes << nodeId << '\t' << formatDouble(point.x()) << '\t' << formatDouble(point.y()) << '\n';
    }

    auto cells = openTable(stem, "cells");
    cells << CELL_HEADER;
    for (std::size_t elementId : sortedKeys(mesh.getElements()))
    {
        const auto* element = mesh.getElement(elementId);
        writeCellRow(cells, formatElementKind(element->getType()), std::to_string(elementId),
                     joinNodeIds(element->getNodeIds()), NOT_APPLICABLE, NOT_APPLICABLE);
    }

    const auto& segments = mesh.getCurveSegmentManager().getAllSegments();
    for (std::size_t segmentId : sortedKeys(segments))
    {
        const auto& segment = segments.at(segmentId);
        const std::array<std::size_t, 2> segmentNodes = {segment.nodeId1, segment.nodeId2};
        const std::string edgeId = segment.edgeId.empty() ? NOT_APPLICABLE : segment.edgeId;
        writeCellRow(cells, "segment", std::to_string(segmentId), joinNodeIds(segmentNodes), edgeId,
                     formatRole(segment.role));
    }
}

void TsvExporter::writeDiscretization(const Meshing::DiscretizationResult3D& discretization,
                                      const std::string& stem)
{
    auto nodes = openTable(stem, "nodes");
    nodes << "node_id\tx\ty\tz\tgeometry_ids\tedge_parameters\n";
    for (std::size_t i = 0; i < discretization.points.size(); ++i)
    {
        const auto& point = discretization.points[i];
        const auto geometryIds = joinWithCommas(discretization.geometryIds[i], [](const std::string& id)
                                                { return id; });
        const auto parameters = joinWithCommas(discretization.edgeParameters[i], formatDouble);
        nodes << i << '\t' << formatDouble(point.x()) << '\t' << formatDouble(point.y()) << '\t'
              << formatDouble(point.z()) << '\t' << geometryIds << '\t' << parameters << '\n';
    }

    auto cells = openTable(stem, "cells");
    cells << CELL_HEADER;
    std::size_t cornerIndex = 0;
    for (const auto& [cornerId, pointIndex] : discretization.cornerIdToPointIndexMap)
    {
        writeCellRow(cells, "corner", std::to_string(cornerIndex++), std::to_string(pointIndex), cornerId,
                     NOT_APPLICABLE);
    }
    writeNodeChains(cells, "edge", discretization.edgeIdToPointIndicesMap);
    writeNodeChains(cells, "surface_points", discretization.surfaceIdToPointIndicesMap);
}

void TsvExporter::writeSurfaceMesh(const Meshing::DiscretizationResult3D& discretization,
                                   const std::vector<Meshing::ConstrainedSubfacet3D>& subfacets,
                                   const std::string& stem)
{
    writePointNodes(discretization.points, stem);

    auto cells = openTable(stem, "cells");
    cells << CELL_HEADER;
    for (std::size_t i = 0; i < subfacets.size(); ++i)
    {
        const auto& subfacet = subfacets[i];
        const std::array<std::size_t, 3> triangleNodes = {subfacet.nodeId1, subfacet.nodeId2, subfacet.nodeId3};
        writeCellRow(cells, "triangle", std::to_string(i), joinNodeIds(triangleNodes), subfacet.geometryId,
                     formatRole(subfacet.role));
    }
}

void TsvExporter::writeSurfaceMesh(const Meshing::SurfaceMesh3D& surfaceMesh, const std::string& stem)
{
    writePointNodes(surfaceMesh.nodes, stem);

    auto cells = openTable(stem, "cells");
    cells << CELL_HEADER;
    writeTriangles(cells, surfaceMesh.triangles, surfaceMesh.faceTriangleIds);
    writeNodeChains(cells, "edge", surfaceMesh.edgeNodeIds);
}

void TsvExporter::writeVolumeMesh(const Meshing::VolumeMesh3D& volumeMesh, const std::string& stem)
{
    writePointNodes(volumeMesh.nodes, stem);

    auto cells = openTable(stem, "cells");
    cells << CELL_HEADER;
    for (std::size_t i = 0; i < volumeMesh.tetrahedra.size(); ++i)
    {
        writeCellRow(cells, "tetrahedron", std::to_string(i), joinNodeIds(volumeMesh.tetrahedra[i]),
                     NOT_APPLICABLE, NOT_APPLICABLE);
    }
    writeTriangles(cells, volumeMesh.boundaryTriangles, volumeMesh.boundaryFaceTriangleIds);
    writeNodeChains(cells, "edge", volumeMesh.boundaryEdgeNodeIds);
}

} // namespace Export
