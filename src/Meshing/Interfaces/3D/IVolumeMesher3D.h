#pragma once

namespace Meshing
{
struct BoundaryDefectReport3D;
struct VolumeMesh3D;

/**
 * @brief Strategy interface for algorithms that mesh a CAD solid's interior volume.
 *
 * Implemented today by RCDTMesher; lets a top-level dispatcher (VolumeMesher3D)
 * hold an implementation without depending on which algorithm it is.
 */
class IVolumeMesher3D
{
public:
    virtual ~IVolumeMesher3D() = default;

    virtual void meshVolume() = 0;

    /// The mesh built by meshVolume(); nullptr until it has run.
    virtual const VolumeMesh3D* getVolumeMesh() const = 0;

    /// The boundary defects of the last run's mesh, empty for a clean one;
    /// nullptr until a run has audited its boundary. Still set when
    /// meshVolume() refused a boundary with holes, with no node IDs then, as
    /// no mesh exists.
    virtual const BoundaryDefectReport3D* getBoundaryDefects() const = 0;
};

} // namespace Meshing
