#pragma once

#include <memory>

namespace Geometry
{
class GeometryCollection;
}
namespace Topology
{
class Topology;
}

namespace Meshing
{
class MeshData;
class MeshConnectivity;
class MeshOperations;

// Central orchestrator holding references to geometry and topology and
// owning mesh-specific mutable data structures.
class MeshingContext
{
public:
    MeshingContext(const Geometry::GeometryCollection& geometry,
                   const Topology::Topology& topology);
    ~MeshingContext();

    // Non-owning references to input domains
    const Geometry::GeometryCollection& getGeometry() const { return geometry_; }
    const Topology::Topology& getTopology() const { return topology_; }

    // Access to mesh data structures
    MeshData& getMeshData();
    MeshConnectivity& getConnectivity();
    MeshOperations& getOperations();

    // Utility: rebuild connectivity after large changes
    void rebuildConnectivity();

    // Clear mesh (data + connectivity)
    void clearMesh();

private:
    const Geometry::GeometryCollection& geometry_;
    const Topology::Topology& topology_;

    std::unique_ptr<MeshData> meshData_;
    std::unique_ptr<MeshConnectivity> connectivity_;
    std::unique_ptr<MeshOperations> operations_;

    void ensureInitialized_();
};

} // namespace Meshing
