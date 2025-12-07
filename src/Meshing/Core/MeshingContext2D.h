#pragma once

#include <memory>
#include <string>

#include "Geometry2D/GeometryCollection2D.h"
#include "Topology2D/Topology2D.h"

namespace Geometry
{
class Surface;
class GeometryCollection;
} // namespace Geometry

namespace Topology
{
class Surface;
class Topology;
} // namespace Topology

namespace Meshing
{

class MeshData2D;
class MeshOperations2D;

/**
 * @brief Central orchestrator for 2D meshing in parametric space
 *
 * Similar to MeshingContext but for 2D domains. Can be created standalone
 * or extracted from a 3D surface using the fromSurface factory method.
 */
class MeshingContext2D
{
public:
    /**
     * @brief Create a 2D meshing context from a 3D surface
     *
     * Extracts the 2D parametric domain from a 3D surface:
     * - Projects boundary corners to (u,v) parametric coordinates
     * - Creates linear edges in parametric space
     * - Sets up topology with boundary loop
     *
     * @param surface The 3D surface geometry
     * @param topoSurface The 3D surface topology
     * @param fullTopology Full 3D topology for edge/corner lookups
     * @param fullGeometry Full 3D geometry for edge/corner coordinates
     * @return MeshingContext2D for the surface's parametric domain
     */
    static MeshingContext2D fromSurface(
        const Geometry::Surface& surface,
        const Topology::Surface& topoSurface,
        const Topology::Topology& fullTopology,
        const Geometry::GeometryCollection& fullGeometry);

    /**
     * @brief Create a standalone 2D meshing context
     */
    MeshingContext2D(std::unique_ptr<Geometry2D::GeometryCollection2D> geometry,
                     std::unique_ptr<Topology2D::Topology2D> topology);

    ~MeshingContext2D();

    // Prevent copying
    MeshingContext2D(const MeshingContext2D&) = delete;
    MeshingContext2D& operator=(const MeshingContext2D&) = delete;

    // Allow moving
    MeshingContext2D(MeshingContext2D&&) noexcept;
    MeshingContext2D& operator=(MeshingContext2D&&) noexcept;

    // Access to geometry and topology
    const Geometry2D::GeometryCollection2D& getGeometry() const { return *geometry_; }
    const Topology2D::Topology2D& getTopology() const { return *topology_; }

    // Access to mesh data structures
    MeshData2D& getMeshData();
    MeshOperations2D& getOperations();

    // Clear mesh (data only, keeps geometry/topology)
    void clearMesh();

private:
    std::unique_ptr<Geometry2D::GeometryCollection2D> geometry_;
    std::unique_ptr<Topology2D::Topology2D> topology_;

    std::unique_ptr<MeshData2D> meshData_;
    std::unique_ptr<MeshOperations2D> operations_;

    void ensureInitialized_();
};

} // namespace Meshing
