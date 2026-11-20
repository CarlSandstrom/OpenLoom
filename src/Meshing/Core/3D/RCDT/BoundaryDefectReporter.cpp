#include "Meshing/Core/3D/RCDT/BoundaryDefectReporter.h"

#include "Common/Exceptions/GeometryException.h"
#include "Common/Exceptions/MeshException.h"
#include "Meshing/Core/3D/RCDT/RestrictedFaceAudit.h"
#include "Meshing/Data/3D/MeshData3D.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <string>

namespace Meshing
{

namespace
{

BoundaryDefectKind3D toKind(RestrictedEdgeDefect defect);
Point3D nodeCoordinates(const MeshData3D& meshData, NodeId nodeId);
NodeId renumberNode(const std::vector<NodeId>& renumbering, NodeId nodeId);

} // namespace

std::vector<NonManifoldRestrictedEdge> BoundaryDefectReporter::audit(const RestrictedFaceMap& restrictedFaces,
                                                                     const Topology3D::Topology3D& topology,
                                                                     const MeshData3D& meshData)
{
    return RestrictedFaceAudit::findNonManifoldEdges(
        restrictedFaces, RestrictedFaceAudit::buildEdgeToAdjacentSurfaces(topology), meshData);
}

BoundaryDefectReport3D BoundaryDefectReporter::toReport(const std::vector<NonManifoldRestrictedEdge>& defects,
                                                        const MeshData3D& meshData,
                                                        const std::optional<std::vector<NodeId>>& renumbering)
{
    BoundaryDefectReport3D report;
    for (auto& defect : defects)
    {
        auto node0 = defect.edge.nodeIds[0];
        auto node1 = defect.edge.nodeIds[1];
        BoundaryDefect3D entry;
        entry.kind = toKind(defect.defect);
        entry.surfaceId = defect.surfaceId;
        entry.endpoints = {nodeCoordinates(meshData, node0), nodeCoordinates(meshData, node1)};
        if (renumbering)
            entry.nodeIds = std::array<NodeId, 2>{renumberNode(*renumbering, node0), renumberNode(*renumbering, node1)};
        report.defects.push_back(std::move(entry));
    }
    return report;
}

size_t BoundaryDefectReporter::count(const BoundaryDefectReport3D& report, BoundaryDefectKind3D kind)
{
    return static_cast<size_t>(std::count_if(report.defects.begin(), report.defects.end(),
                                             [kind](const BoundaryDefect3D& defect)
                                             { return defect.kind == kind; }));
}

void BoundaryDefectReporter::logSummary(const BoundaryDefectReport3D& report)
{
    if (report.defects.empty()) return;

    spdlog::warn("RCDTMesher: boundary defects -- {} missing face, {} excess face, {} surface mismatch",
                 count(report, BoundaryDefectKind3D::MissingFace),
                 count(report, BoundaryDefectKind3D::ExcessFace),
                 count(report, BoundaryDefectKind3D::SurfaceMismatch));
}

namespace
{

BoundaryDefectKind3D toKind(RestrictedEdgeDefect defect)
{
    switch (defect)
    {
    case RestrictedEdgeDefect::MissingFace:
        return BoundaryDefectKind3D::MissingFace;
    case RestrictedEdgeDefect::ExcessFace:
        return BoundaryDefectKind3D::ExcessFace;
    case RestrictedEdgeDefect::SurfaceMismatch:
        return BoundaryDefectKind3D::SurfaceMismatch;
    }
    OPENLOOM_THROW_MESH(INVALID_OPERATION, "BoundaryDefectReporter: unknown RestrictedEdgeDefect");
}

Point3D nodeCoordinates(const MeshData3D& meshData, NodeId nodeId)
{
    auto node = meshData.getNode(nodeId);
    if (!node) OPENLOOM_THROW_ENTITY_NOT_FOUND("Node", std::to_string(nodeId.value()));
    return node->getCoordinates();
}

// Every defect edge belongs to a restricted face the output mesh carries as a
// triangle, so its nodes always survive the renumbering.
NodeId renumberNode(const std::vector<NodeId>& renumbering, NodeId nodeId)
{
    if (nodeId.value() >= renumbering.size() || renumbering[nodeId.value()] == NodeId::INVALID)
    {
        OPENLOOM_THROW_MESH(INVALID_OPERATION,
                            "BoundaryDefectReporter: defect node " + std::to_string(nodeId.value()) +
                                " is not in the output mesh");
    }
    return renumbering[nodeId.value()];
}

} // namespace

} // namespace Meshing
