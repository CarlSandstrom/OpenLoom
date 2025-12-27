#include "GeometryOperations2D.h"
#include "Topology2D/Topology2D.h"

namespace Geometry2D
{

using Meshing::Point2D;

GeometryOperations2D::GeometryOperations2D(const GeometryCollection2D& geometry)
    : geometry_(geometry)
{
}

GeometryOperations2D::PointExtractionResult
GeometryOperations2D::extractCornerPoints() const
{
    PointExtractionResult result;

    for (const auto& cornerId : geometry_.getAllCornerIds())
    {
        const auto* corner = geometry_.getCorner(cornerId);
        result.points.push_back(corner->getPoint());
        result.cornerIdToPointIndexMap[cornerId] = result.points.size() - 1;
    }

    return result;
}

GeometryOperations2D::PointExtractionResult
GeometryOperations2D::extractPointsWithEdgeDiscretization(
    const Topology2D::Topology2D& topology,
    const DiscretizationSettings2D& settings) const
{
    // Start with corner points
    PointExtractionResult result = extractCornerPoints();

    // Add intermediate points along edges based on discretization settings
    size_t numSegments = settings.getNumSegmentsPerEdge();

    for (const auto& edgeId : topology.getAllEdgeIds())
    {
        const auto& edgeTopology = topology.getEdge(edgeId);
        const auto* edgeGeometry = geometry_.getEdge(edgeId);
        auto parameterBounds = edgeGeometry->getParameterBounds();

        // Get start and end corner point indices
        size_t startCornerIdx = result.cornerIdToPointIndexMap.at(edgeTopology.getStartCornerId());
        size_t endCornerIdx = result.cornerIdToPointIndexMap.at(edgeTopology.getEndCornerId());

        // Initialize the edge point indices list with the start corner
        std::vector<size_t> edgePointIndices;
        edgePointIndices.push_back(startCornerIdx);

        // Add intermediate points if discretization is enabled
        if (numSegments > 1)
        {
            for (size_t i = 1; i < numSegments; ++i)
            {
                double t = parameterBounds.first +
                          i * (parameterBounds.second - parameterBounds.first) / numSegments;
                Point2D point = edgeGeometry->getPoint(t);

                size_t pointIndex = result.points.size();
                result.points.push_back(point);
                edgePointIndices.push_back(pointIndex);
            }
        }

        // Add the end corner
        edgePointIndices.push_back(endCornerIdx);

        // Store the edge point indices
        result.edgeIdToPointIndicesMap[edgeId] = edgePointIndices;
    }

    return result;
}

} // namespace Geometry2D
