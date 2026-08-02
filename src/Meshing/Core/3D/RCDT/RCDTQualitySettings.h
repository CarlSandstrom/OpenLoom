#pragma once

#include <cstddef>
#include <optional>

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

    /// Floor on a restricted triangle's shortest edge below which it is left
    /// unrefined even if it still fails the quality criteria above. Without
    /// this, a triangle whose ratio sits just past the threshold can have its
    /// circumcenter land within roughly one edge-length of its own vertices,
    /// producing an equally-bad, slightly smaller sliver next to it every
    /// iteration — a non-terminating cascade. If unset, RCDTRefiner derives
    /// it from the initial boundary discretization: the median
    /// nearest-neighbor distance among the initial nodes, divided by 10.
    std::optional<double> minimumEdgeLength;
};

} // namespace Meshing
