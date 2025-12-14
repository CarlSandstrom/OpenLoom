#pragma once

#include <memory>

namespace Geometry3D
{
class GeometryCollection3D;
}
namespace Topology3D
{
class Topology3D;
}

namespace Meshing
{
class MeshData;
class MeshConnectivity;
class MeshOperations;

// Central orchestrator holding references to geometry and topology and
// owning mesh-specific mutable data structures.
class MeshingContext3D
{
public:
    MeshingContext3D(const Geometry3D::GeometryCollection3D& geometry,
                     const Topology3D::Topology3D& topology);
    ~MeshingContext3D();

    // Non-owning references to input domains
    const Geometry3D::GeometryCollection3D& getGeometry() const { return geometry_; }
    const Topology3D::Topology3D& getTopology() const { return topology_; }

    // Access to mesh data structures
    MeshData& getMeshData();
    MeshConnectivity& getConnectivity();
    MeshOperations& getOperations();

    // Utility: rebuild connectivity after large changes
    void rebuildConnectivity();

    // Clear mesh (data + connectivity)
    void clearMesh();

private:
    const Geometry3D::GeometryCollection3D& geometry_;
    const Topology3D::Topology3D& topology_;

    std::unique_ptr<MeshData> meshData_;
    std::unique_ptr<MeshConnectivity> connectivity_;
    std::unique_ptr<MeshOperations> operations_;

    void ensureInitialized_();
};

} // namespace Meshing
