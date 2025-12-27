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

    if (numSegments > 1)
    {
        for (const auto& edgeId : topology.getAllEdgeIds())
        {
            const auto* edgeGeometry = geometry_.getEdge(edgeId);
            auto parameterBounds = edgeGeometry->getParameterBounds();

            // Add intermediate points (skip i=0 and i=numSegments as they are corners)
            for (size_t i = 1; i < numSegments; ++i)
            {
                double t = parameterBounds.first +
                          i * (parameterBounds.second - parameterBounds.first) / numSegments;
                Point2D point = edgeGeometry->getPoint(t);
                result.points.push_back(point);
            }
        }
    }

    return result;
}

} // namespace Geometry2D
