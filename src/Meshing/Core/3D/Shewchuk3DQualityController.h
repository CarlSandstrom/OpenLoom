#pragma once

#include "Meshing/Interfaces/IQualityController3D.h"
#include "ElementGeometry3D.h"
#include "ElementQuality3D.h"
#include <cstddef>
#include <memory>

namespace Meshing
{
class MeshData3D;

/**
 * @brief Quality controller implementing Shewchuk's 3D mesh quality criteria
 *
 * Based on Shewchuk's paper "Tetrahedral Mesh Generation by Delaunay Refinement",
 * this controller uses the circumradius-to-shortest-edge ratio (B) as the primary
 * quality metric. A tetrahedron is considered "skinny" if its B ratio exceeds
 * the specified bound (typically B > 2).
 *
 * Note: This implementation focuses on the B ratio. Slivers (tetrahedra with
 * good B ratios but poor dihedral angles) are not directly detected, as per
 * the paper's observation that they typically arise in small numbers and can
 * often be removed through additional refinement or post-processing.
 */
class Shewchuk3DQualityController : public IQualityController3D
{
public:
    /**
     * @brief Construct a quality controller with Shewchuk's criteria
     *
     * @param meshData The mesh data to evaluate
     * @param circumradiusToShortestEdgeRatioBound The B bound (must be > 2 for guaranteed termination)
     * @param elementLimit Maximum number of tetrahedra allowed
     */
    Shewchuk3DQualityController(const MeshData3D& meshData,
                                double circumradiusToShortestEdgeRatioBound,
                                std::size_t elementLimit);

    bool isMeshAcceptable(const MeshData3D& data, const MeshConnectivity& connectivity) const override;
    bool isTetrahedronAcceptable(const TetrahedralElement& element) const override;
    double getTargetElementQuality() const override;
    std::size_t getElementLimit() const override;

    /**
     * @brief Get the circumradius-to-shortest-edge ratio bound
     * @return The B bound value
     */
    double getCircumradiusToShortestEdgeRatioBound() const { return circumradiusToShortestEdgeRatioBound_; }

    /**
     * @brief Check if a tetrahedron is too small to refine reliably
     * @param element The tetrahedron to check
     * @return true if the tetrahedron is below minimum refinable size
     */
    bool isTetrahedronTooSmall(const TetrahedralElement& element) const override;

    // Minimum thresholds for refinable tetrahedra
    static constexpr double MIN_REFINABLE_VOLUME = 1e-18;
    static constexpr double MIN_REFINABLE_EDGE = 1e-10;

private:
    std::unique_ptr<ElementGeometry3D> geometry_;
    std::unique_ptr<ElementQuality3D> quality_;
    double circumradiusToShortestEdgeRatioBound_;
    std::size_t elementLimit_;
};

} // namespace Meshing
