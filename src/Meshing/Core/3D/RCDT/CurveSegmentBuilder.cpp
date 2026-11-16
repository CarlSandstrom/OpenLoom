#include "Meshing/Core/3D/RCDT/CurveSegmentBuilder.h"

#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/IEdge3D.h"
#include "Meshing/Data/3D/DiscretizationResult3D.h"
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
                                        double parameterMin,
                                        double parameterMax)
{
    std::vector<double> parameters;
    parameters.reserve(pointIndices.size());
    for (size_t position = 0; position < pointIndices.size(); ++position)
    {
        if (position == 0)
            parameters.push_back(parameterMin);
        else if (position == pointIndices.size() - 1)
            parameters.push_back(parameterMax);
        else
            parameters.push_back(discretization.edgeParameters[pointIndices[position]][0]);
    }
    return parameters;
}

} // namespace

CurveSegmentManager CurveSegmentBuilder::build(const Topology3D::Topology3D& topology,
                                               const Geometry3D::GeometryCollection3D& geometry,
                                               const DiscretizationResult3D& discretization,
                                               const std::map<size_t, NodeId>& pointIndexToNodeIdMap)
{
    CurveSegmentManager manager;
    auto& seams = topology.getSeamCollection();

    for (auto& edgeId : topology.getAllEdgeIds())
    {
        if (seams.isSeamTwin(edgeId))
            continue;

        auto geometryEdge = geometry.getEdge(edgeId);
        if (!geometryEdge)
            continue;

        // A degenerate edge (e.g. a sphere's polar edge) is a zero-length
        // topological placeholder, not a real curve — its start and end are
        // the same point. Building a "segment" for it would fabricate a
        // constraint with a nonzero parameter range but no actual length.
        if (geometryEdge->isDegenerate())
            continue;

        auto sequenceIterator = discretization.edgeIdToPointIndicesMap.find(edgeId);
        if (sequenceIterator == discretization.edgeIdToPointIndicesMap.end())
            continue;

        auto& pointIndices = sequenceIterator->second;
        if (pointIndices.size() < 2)
            continue;

        auto [parameterMin, parameterMax] = geometryEdge->getParameterBounds();
        auto parameters = parametersAlongEdge(pointIndices, discretization, parameterMin, parameterMax);

        for (size_t index = 0; index + 1 < pointIndices.size(); ++index)
        {
            auto node0 = pointIndexToNodeIdMap.find(pointIndices[index]);
            auto node1 = pointIndexToNodeIdMap.find(pointIndices[index + 1]);
            if (node0 == pointIndexToNodeIdMap.end() || node1 == pointIndexToNodeIdMap.end())
                continue;

            CurveSegment segment;
            segment.nodeId1 = node0->second;
            segment.nodeId2 = node1->second;
            segment.edgeId = edgeId;
            segment.tStart = parameters[index];
            segment.tEnd = parameters[index + 1];
            manager.addSegment(segment);
        }
    }
    return manager;
}

} // namespace Meshing
