#pragma once

#include "Common/Types.h"
#include "Meshing/Core/3D/RCDT/RestrictedFaceTypes.h"
#include "Meshing/Data/3D/BoundaryDefectReport3D.h"

#include <optional>
#include <vector>

namespace Topology3D
{
class Topology3D;
} // namespace Topology3D

namespace Meshing
{

class MeshData3D;

/**
 * @brief Turns RestrictedFaceAudit's findings into the BoundaryDefectReport3D
 * RCDTMesher hands its caller.
 *
 * The audit names edges by the working mesh's node IDs; the output mesh is
 * renumbered by UnreferencedNodeRemover, so toReport() translates them when a
 * mesh was produced and leaves them unset when none was.
 */
class BoundaryDefectReporter
{
public:
    static std::vector<NonManifoldRestrictedEdge> audit(const RestrictedFaceMap& restrictedFaces,
                                                        const Topology3D::Topology3D& topology,
                                                        const MeshData3D& meshData);

    /// Endpoint coordinates are read from meshData as it stands. renumbering
    /// is UnreferencedNodeRemover's old-to-new map for the output mesh.
    static BoundaryDefectReport3D toReport(const std::vector<NonManifoldRestrictedEdge>& defects,
                                           const MeshData3D& meshData,
                                           const std::optional<std::vector<NodeId>>& renumbering = std::nullopt);

    static size_t count(const BoundaryDefectReport3D& report, BoundaryDefectKind3D kind);

    /// One warning with the count of each kind; nothing for a clean report.
    static void logSummary(const BoundaryDefectReport3D& report);
};

} // namespace Meshing
