#pragma once

namespace Meshing
{
class TetrahedralElement;

/**
 * @brief Interface for 3D mesh quality control
 *
 * Defines the contract for mesh quality evaluation in 3D tetrahedral meshes.
 * Implementations can use different quality metrics (circumradius-to-shortest-edge
 * ratio, dihedral angles, etc.) to determine mesh acceptability.
 */
class IQualityController3D
{
public:
    virtual ~IQualityController3D() = default;

    /**
     * @brief Check if a single tetrahedron meets quality requirements
     * @param element The tetrahedron to evaluate
     * @return true if the tetrahedron meets quality requirements
     */
    virtual bool isTetrahedronAcceptable(const TetrahedralElement& element) const = 0;

    /**
     * @brief Get the target element quality bound
     * @return The circumradius-to-shortest-edge ratio bound (B value)
     */
    virtual double getTargetElementQuality() const = 0;

    /**
     * @brief Check if a tetrahedron is too small to refine reliably
     * @param element The tetrahedron to check
     * @return true if the tetrahedron is below minimum refinable size
     */
    virtual bool isTetrahedronTooSmall(const TetrahedralElement& element) const = 0;
};

} // namespace Meshing
