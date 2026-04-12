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
                        const std::map<std::string, std::vector<size_t>>& edgeIdToPointIndicesMap,
                        const std::map<size_t, size_t>& pointIndexToNodeIdMap,
                        const std::vector<std::vector<double>>& edgeParameters)
{
    manager.clear();

    const auto& seams = topology.getSeamCollection();

    for (const auto& edgeId : topology.getAllEdgeIds())
    {
        if (seams.isSeamTwin(edgeId))
            continue;

        const Geometry3D::IEdge3D* geometryEdge = geometry.getEdge(edgeId);
        if (!geometryEdge)
            continue;

        const auto sequenceIt = edgeIdToPointIndicesMap.find(edgeId);
        if (sequenceIt == edgeIdToPointIndicesMap.end())
            continue;

        const auto& pointIndices = sequenceIt->second;
        if (pointIndices.size() < 2)
            continue;

        const auto [tMin, tMax] = geometryEdge->getParameterBounds();

        // t-values: first position uses tMin, last uses tMax, interior positions
        // read the stored edge parameter (one value per interior point).
        std::vector<double> tValues;
        tValues.reserve(pointIndices.size());
        for (size_t position = 0; position < pointIndices.size(); ++position)
        {
            if (position == 0)
                tValues.push_back(tMin);
            else if (position == pointIndices.size() - 1)
                tValues.push_back(tMax);
            else
                tValues.push_back(edgeParameters[pointIndices[position]][0]);
        }

        // One segment per consecutive node pair.
        for (size_t i = 0; i + 1 < pointIndices.size(); ++i)
        {
            const auto nodeIt1 = pointIndexToNodeIdMap.find(pointIndices[i]);
            const auto nodeIt2 = pointIndexToNodeIdMap.find(pointIndices[i + 1]);
            if (nodeIt1 == pointIndexToNodeIdMap.end() || nodeIt2 == pointIndexToNodeIdMap.end())
                continue;

            CurveSegment segment;
            segment.nodeId1 = nodeIt1->second;
            segment.nodeId2 = nodeIt2->second;
            segment.edgeId = edgeId;
            segment.tStart = tValues[i];
            segment.tEnd = tValues[i + 1];
            manager.addSegment(segment);
        }
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
