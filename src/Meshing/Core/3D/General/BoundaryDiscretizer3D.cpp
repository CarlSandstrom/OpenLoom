#include "Meshing/Core/3D/General/BoundaryDiscretizer3D.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ICorner3D.h"
#include "Geometry/3D/Base/IEdge3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Core/3D/General/MeshingContext3D.h"
#include "Topology/Topology3D.h"

namespace Meshing
{

BoundaryDiscretizer3D::BoundaryDiscretizer3D(const MeshingContext3D& context,
                                             const Geometry3D::DiscretizationSettings3D& settings) :
    context_(&context),
    settings_(settings)
{
}

DiscretizationResult3D BoundaryDiscretizer3D::discretize() const
{
    DiscretizationResult3D result;

    const auto* geometry = context_->getGeometry();
    const auto* topology = context_->getTopology();

    if (!geometry || !topology)
    {
        return result;
    }

    // Step 1: Sample corner points
    for (const auto& cornerId : topology->getAllCornerIds())
    {
        const auto* corner = geometry->getCorner(cornerId);
        if (!corner)
        {
            continue;
        }

        size_t pointIndex = result.points.size();
        result.points.push_back(corner->getPoint());
        result.edgeParameters.push_back({});
        result.geometryIds.push_back({cornerId});
        result.cornerIdToPointIndexMap[cornerId] = pointIndex;
    }

    // Step 2: Sample edge interior points (excluding endpoints)
    size_t numSegments = settings_.getNumSegmentsPerEdge();

    for (const auto& edgeId : topology->getAllEdgeIds())
    {
        const auto& topoEdge = topology->getEdge(edgeId);
        const auto* edge = geometry->getEdge(edgeId);

        if (!edge)
        {
            continue;
        }

        auto [tMin, tMax] = edge->getParameterBounds();

        // Get start and end corner point indices
        size_t startIdx = result.cornerIdToPointIndexMap.at(topoEdge.getStartCornerId());
        size_t endIdx = result.cornerIdToPointIndexMap.at(topoEdge.getEndCornerId());

        // Add edge parameters to corner points
        result.edgeParameters[startIdx].push_back(tMin);
        result.geometryIds[startIdx].push_back(edgeId);
        result.edgeParameters[endIdx].push_back(tMax);
        result.geometryIds[endIdx].push_back(edgeId);

        // Build the edge point indices list starting with start corner
        std::vector<size_t> edgePointIndices;
        edgePointIndices.push_back(startIdx);

        // Add interior samples
        for (size_t i = 1; i < numSegments; ++i)
        {
            double t = tMin + (tMax - tMin) * static_cast<double>(i) / static_cast<double>(numSegments);
            Point3D point = edge->getPoint(t);

            size_t pointIndex = result.points.size();
            result.points.push_back(point);
            result.edgeParameters.push_back({t});
            result.geometryIds.push_back({edgeId});
            edgePointIndices.push_back(pointIndex);
        }

        // Add end corner
        edgePointIndices.push_back(endIdx);

        result.edgeIdToPointIndicesMap[edgeId] = edgePointIndices;
    }

    // Step 3: Sample surface interior points
    size_t surfaceSamples = settings_.getNumSamplesPerSurfaceDirection();

    for (const auto& surfaceId : topology->getAllSurfaceIds())
    {
        const auto* surface = geometry->getSurface(surfaceId);

        if (!surface)
        {
            continue;
        }

        auto bounds = surface->getParameterBounds();
        double uMin = bounds.getUMin();
        double uMax = bounds.getUMax();
        double vMin = bounds.getVMin();
        double vMax = bounds.getVMax();

        std::vector<size_t> surfacePointIndices;

        // Sample interior points only (boundary is sampled via edges)
        for (size_t i = 1; i < surfaceSamples; ++i)
        {
            double u = uMin + (uMax - uMin) * static_cast<double>(i) / static_cast<double>(surfaceSamples);
            for (size_t j = 1; j < surfaceSamples; ++j)
            {
                double v = vMin + (vMax - vMin) * static_cast<double>(j) / static_cast<double>(surfaceSamples);
                Point3D point = surface->getPoint(u, v);

                size_t pointIndex = result.points.size();
                result.points.push_back(point);
                result.edgeParameters.push_back({});
                result.geometryIds.push_back({surfaceId});
                surfacePointIndices.push_back(pointIndex);
            }
        }

        result.surfaceIdToPointIndicesMap[surfaceId] = surfacePointIndices;
    }

    return result;
}

} // namespace Meshing
