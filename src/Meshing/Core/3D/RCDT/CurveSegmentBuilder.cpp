#include "Meshing/Core/3D/RCDT/CurveSegmentBuilder.h"

#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/IEdge3D.h"
#include "Meshing/Core/3D/General/DiscretizationResult3D.h"
#include "Topology/SeamCollection.h"
#include "Topology/Topology3D.h"

#include <vector>

namespace Meshing
{

namespace
{

// The edge parameter of each sampled point, in the order the points run along the
// edge. The two endpoints take the edge's own parameter bounds; every interior point
// reads the parameter the discretizer stored for it (one value per interior point).
std::vector<double> parametersAlongEdge(const std::vector<size_t>& pointIndices,
                                        const DiscretizationResult3D& discretization,
                                        double tMin,
                                        double tMax)
{
    std::vector<double> tValues;
    tValues.reserve(pointIndices.size());
    for (size_t position = 0; position < pointIndices.size(); ++position)
    {
        if (position == 0)
            tValues.push_back(tMin);
        else if (position == pointIndices.size() - 1)
            tValues.push_back(tMax);
        else
            tValues.push_back(discretization.edgeParameters[pointIndices[position]][0]);
    }
    return tValues;
}

} // namespace

CurveSegmentManager CurveSegmentBuilder::build(const Topology3D::Topology3D& topology,
                                               const Geometry3D::GeometryCollection3D& geometry,
                                               const DiscretizationResult3D& discretization,
                                               const std::map<size_t, size_t>& pointIndexToNodeIdMap)
{
    CurveSegmentManager manager;
    const auto& seams = topology.getSeamCollection();

    for (const auto& edgeId : topology.getAllEdgeIds())
    {
        if (seams.isSeamTwin(edgeId))
            continue;

        const Geometry3D::IEdge3D* geometryEdge = geometry.getEdge(edgeId);
        if (!geometryEdge)
            continue;

        // A degenerate edge (e.g. a sphere's polar edge) is a zero-length
        // topological placeholder, not a real curve — its start and end are
        // the same point. Building a "segment" for it would fabricate a
        // constraint with a nonzero parameter range but no actual length.
        if (geometryEdge->isDegenerate())
            continue;

        const auto sequenceIt = discretization.edgeIdToPointIndicesMap.find(edgeId);
        if (sequenceIt == discretization.edgeIdToPointIndicesMap.end())
            continue;

        const auto& pointIndices = sequenceIt->second;
        if (pointIndices.size() < 2)
            continue;

        const auto [tMin, tMax] = geometryEdge->getParameterBounds();
        const std::vector<double> tValues = parametersAlongEdge(pointIndices, discretization, tMin, tMax);

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
    return manager;
}

} // namespace Meshing
