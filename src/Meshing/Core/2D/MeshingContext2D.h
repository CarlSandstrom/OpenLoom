#pragma once

#include <memory>
#include <optional>
#include <string>

#include "Geometry/2D/Base/GeometryCollection2D.h"
#include "Meshing/Core/2D/PeriodicDomain2D.h"
#include "Topology2D/Topology2D.h"

namespace Meshing
{

class MeshData2D;
class MeshMutator2D;
class MeshOperations2D;
class PeriodicMeshData2D;

/**
 * @brief Central orchestrator for 2D meshing in parametric space
 *
 * Similar to MeshingContext3D but for 2D domains.
 */
class MeshingContext2D
{
public:
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
    const MeshData2D& getMeshData() const;
    MeshOperations2D& getOperations();

    /// Configure the context for periodic meshing.
    /// Must be called before the first call to getMeshData() or getOperations().
    /// Nothing calls this yet: it is the entry point to the periodic-offset
    /// layer (PeriodicMeshData2D), kept as groundwork for periodic meshing.
    void setPeriodicConfig(const PeriodicDomainConfig& config);

    /// Returns the periodic data layer, or nullptr for non-periodic contexts.
    PeriodicMeshData2D* getPeriodicData() const { return periodicData_.get(); }

private:
    std::unique_ptr<Geometry2D::GeometryCollection2D> geometry_;
    std::unique_ptr<Topology2D::Topology2D> topology_;

    std::optional<PeriodicDomainConfig> periodicConfig_;

    std::unique_ptr<MeshData2D> meshData_;
    std::unique_ptr<PeriodicMeshData2D> periodicData_;
    std::unique_ptr<MeshMutator2D> meshMutator_;
    std::unique_ptr<MeshOperations2D> meshOperations_;

    void ensureInitialized();
};

} // namespace Meshing
