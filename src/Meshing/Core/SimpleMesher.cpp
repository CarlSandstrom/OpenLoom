#include "SimpleMesher.h"
#include "../Data/MeshOperations.h"
#include "../Data/TetrahedralElement.h"
#include "../Operations/ScopedTransaction.h"
#include "Delaunay3D.h"
#include "Geometry/Base/GeometryCollection.h"
#include "MeshingContext.h"
#include "Topology/Topology.h"

#include <array>
#include <memory>

namespace Meshing
{

void SimpleMesher::generate(MeshingContext& context)
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

    Delaunay3D delaunay(context);
    delaunay.initialize(points);

    // Build connectivity
    context.rebuildConnectivity();
}

} // namespace Meshing
