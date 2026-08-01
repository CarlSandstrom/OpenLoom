#pragma once

#include <cstddef>

namespace Meshing
{

struct RCDTQualitySettings
{
    double maximumCircumradiusToShortestEdgeRatio = 1.0;
    double maximumChordDeviation = 0.1;

    /// Laplacian smoothing sweeps applied to the output surface mesh after
    /// refinement (see SurfaceMeshSmoother). Delaunay refinement alone does
    /// not reliably converge to FEM-quality elements on curved surfaces; set
    /// to 0 to disable.
    std::size_t smoothingIterations = 5;
};

} // namespace Meshing
