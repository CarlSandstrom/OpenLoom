#pragma once

#include "Meshing/Interfaces/IMesher.h"
#include <string>

namespace Meshing
{

/**
 * @brief Mesher that generates constrained Delaunay meshes
 *
 * Respects geometric constraints (edges AND faces) from the input CAD geometry.
 * Uses classical CDT approach to force segments and facets into the mesh.
 *
 * This mesher:
 * - Inserts corner nodes from topology
 * - Samples edges from topology/geometry
 * - Samples and triangulates surfaces
 * - Creates initial Delaunay triangulation
 * - Forces constraint segments into the mesh
 * - Forces constraint facets into the mesh
 */
class ConstrainedMesher : public IMesher
{
public:
    explicit ConstrainedMesher(const IQualityController* qualityController = nullptr,
                               size_t samplesPerEdge = 10,
                               size_t samplesPerSurface = 5) :
        IMesher(qualityController),
        samplesPerEdge_(samplesPerEdge),
        samplesPerSurface_(samplesPerSurface) {}

    ~ConstrainedMesher() override = default;

    void generate(MeshingContext3D& context) override;
    std::string getName() const override { return "ConstrainedMesher"; }

    // Configuration
    void setSamplesPerEdge(size_t samples) { samplesPerEdge_ = samples; }
    size_t getSamplesPerEdge() const { return samplesPerEdge_; }

    void setSamplesPerSurface(size_t samples) { samplesPerSurface_ = samples; }
    size_t getSamplesPerSurface() const { return samplesPerSurface_; }

private:
    size_t samplesPerEdge_;
    size_t samplesPerSurface_;
};

} // namespace Meshing
