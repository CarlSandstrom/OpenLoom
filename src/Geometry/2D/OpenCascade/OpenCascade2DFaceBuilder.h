#pragma once

#include <memory>

namespace Geometry2D {
    class IFace2D;
    class GeometryCollection2D;
}

namespace Topology2D {
    class Topology2D;
}

namespace Geometry2D {

class OpenCascade2DFaceBuilder {
public:
    /**
     * @brief Build a face from topology and geometry
     *
     * Creates an OpenCascade2DFace representing the 2D domain with
     * outer boundary and hole loops from topology.
     *
     * @param topology The topological structure defining boundaries
     * @param geometry The geometric entities (edges, corners)
     * @return Face representing the domain with outer boundary and holes
     */
    static std::unique_ptr<IFace2D> buildFromTopology(
        const Topology2D::Topology2D& topology,
        const GeometryCollection2D& geometry);
};

} // namespace Geometry2D
