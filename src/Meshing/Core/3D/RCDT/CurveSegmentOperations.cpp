#include "Meshing/Core/3D/RCDT/CurveSegmentOperations.h"

#include "Common/Exceptions/GeometryException.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/IEdge3D.h"
#include "Topology/SeamCollection.h"
#include "Topology/Topology3D.h"

namespace Meshing
{

void buildCurveSegments(CurveSegmentManager& manager,
                        const Topology3D::Topology3D& topology,
                        const Geometry3D::GeometryCollection3D& geometry,
                        const std::unordered_map<std::string, size_t>& cornerIdToNodeId)
{
    manager.clear();

    const auto& seams = topology.getSeamCollection();

    for (const auto& edgeId : topology.getAllEdgeIds())
    {
        if (seams.isSeamTwin(edgeId))
            continue;

        const auto& topoEdge = topology.getEdge(edgeId);
        const Geometry3D::IEdge3D* geometryEdge = geometry.getEdge(edgeId);

        if (!geometryEdge)
            continue;

        const auto startIt = cornerIdToNodeId.find(topoEdge.getStartCornerId());
        const auto endIt = cornerIdToNodeId.find(topoEdge.getEndCornerId());

        if (startIt == cornerIdToNodeId.end() || endIt == cornerIdToNodeId.end())
            continue;

        const auto [tStart, tEnd] = geometryEdge->getParameterBounds();

        CurveSegment segment;
        segment.nodeId1 = startIt->second;
        segment.nodeId2 = endIt->second;
        segment.edgeId = edgeId;
        segment.tStart = tStart;
        segment.tEnd = tEnd;

        manager.addSegment(segment);
    }
}

Point3D computeSplitPoint(const CurveSegment& segment,
                          const Geometry3D::GeometryCollection3D& geometry)
{
    const Geometry3D::IEdge3D* edge = geometry.getEdge(segment.edgeId);
    OPENLOOM_REQUIRE_NOT_NULL(edge, segment.edgeId);

    const double tMid = edge->getParameterAtArcLengthFraction(segment.tStart, segment.tEnd, 0.5);
    return edge->getPoint(tMid);
}

} // namespace Meshing
