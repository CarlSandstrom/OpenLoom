#include "OpenCascade2DFaceBuilder.h"
#include "OpenCascade2DEdgeLoop.h"
#include "OpenCascade2DFace.h"
#include "Geometry/2D/Base/GeometryCollection2D.h"
#include "Topology2D/Topology2D.h"

namespace Geometry2D {

std::unique_ptr<IFace2D> OpenCascade2DFaceBuilder::buildFromTopology(
    const Topology2D::Topology2D& topology,
    const GeometryCollection2D& geometry)
{
    // Build outer edge loop
    auto outerLoop = std::make_unique<OpenCascade2DEdgeLoop>(
        topology.getOuterEdgeLoop(), geometry);

    // Build face with outer loop
    auto face = std::make_unique<OpenCascade2DFace>(std::move(outerLoop));

    // Add hole loops
    for (const auto& holeEdgeIds : topology.getHoleEdgeLoops())
    {
        auto holeLoop = std::make_unique<OpenCascade2DEdgeLoop>(
            holeEdgeIds, geometry);
        face->addHole(std::move(holeLoop));
    }

    return face;
}

} // namespace Geometry2D
