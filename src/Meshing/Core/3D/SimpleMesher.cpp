#include "SimpleMesher.h"
#include "../Data/MeshMutator3D.h"
#include "../Data/TetrahedralElement.h"
#include "../Operations/ScopedTransaction.h"
#include "ConstrainedDelaunay3D.h"
#include "Geometry/Base/GeometryCollection3D.h"
#include "MeshingContext3D.h"
#include "Topology/Topology3D.h"

#include <array>
#include <memory>

namespace Meshing
{

void SimpleMesher::generate(MeshingContext3D& context)
{
    // Start with a clean mesh for this simple example
    context.clearMesh();

    auto& ops = context.getOperations();

    // Begin transaction to ensure rollback on any failure
    std::vector<Point3D> points;
    for (auto cornerId : context.getTopology().getAllCornerIds())
    {
        auto geometryCorner = context.getGeometry().getCorner(cornerId);
        auto point = geometryCorner->getPoint();
        points.push_back(point);
    }

    for (auto edgeId : context.getTopology().getAllEdgeIds())
    {
        auto geometryEdge = context.getGeometry().getEdge(edgeId);
        auto parameterRange = geometryEdge->getParameterBounds();
        const size_t numSamples = 10; // Simple uniform sampling
        for (size_t i = 0; i <= numSamples; ++i)
        {
            double t = parameterRange.first + static_cast<double>(i) / static_cast<double>(numSamples) * (parameterRange.second - parameterRange.first);
            auto point = geometryEdge->getPoint(t);
            points.push_back(point);
        }
    }

    /*     ConstrainedDelaunay3D delaunay(context);
        delaunay.initialize(points);

        // Build connectivity
        context.rebuildConnectivity();
     */
}

} // namespace Meshing
