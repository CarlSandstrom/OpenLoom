#pragma once

#include "Meshing/Core/2D/DiscretizationResult2D.h"
#include "Meshing/Core/3D/General/DiscretizationResult3D.h"
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace Geometry3D
{
class ISurface3D;
class GeometryCollection3D;
} // namespace Geometry3D

namespace Topology3D
{
class Surface3D;
class SeamCollection;
} // namespace Topology3D

namespace Meshing
{

/**
 * @brief Builds a UV-space 2D discretization for a single 3D surface face.
 *
 * Takes global 3D point indices belonging to a surface, projects them into
 * UV parametric space, normalises edge parameters to [0,1] for use with
 * LinearEdge2D, and maps all edge/corner connectivity to local (0..N-1) indices.
 *
 * Seam-twin edges are duplicated at U + uPeriod so that periodic surfaces
 * (e.g. cylinders) triangulate correctly in the UV plane.
 */
class FacetDiscretization2DBuilder
{
public:
    FacetDiscretization2DBuilder(const Geometry3D::ISurface3D& surface,
                                 const Topology3D::Surface3D& topoSurface,
                                 const Topology3D::SeamCollection& seams,
                                 const DiscretizationResult3D& discretization3D,
                                 const std::vector<size_t>& globalPointIndices,
                                 std::function<size_t(size_t)> pointIdxToNode3DId,
                                 const Geometry3D::GeometryCollection3D& geometry);

    void build();

    const DiscretizationResult2D& getDiscretization2D() const { return discretization2D_; }
    const std::vector<size_t>& getLocalIndexToNode3DId() const { return localIndexToNode3DId_; }

    DiscretizationResult2D takeDiscretization2D() { return std::move(discretization2D_); }
    std::vector<size_t> takeLocalIndexToNode3DId() { return std::move(localIndexToNode3DId_); }

private:
    // Maps from global point index to the local index of the period-shifted duplicate.
    // Separated by direction because a doubly-periodic surface (torus) may have two twins
    // that each shift a different period, and non-seam edges need to choose the right one.
    struct ShiftedLocalMaps
    {
        std::map<size_t, size_t> uShifted; // global → local for U-period-shifted copies
        std::map<size_t, size_t> vShifted; // global → local for V-period-shifted copies
    };

    void buildGlobalToLocalMap();
    void buildPoints();
    void buildCornerMap();
    void buildEdgeMaps();
    ShiftedLocalMaps processSeamEdges();
    void processNonSeamEdges(const ShiftedLocalMaps& shiftedLocals);

    const Geometry3D::ISurface3D& surface_;
    const Topology3D::Surface3D& topoSurface_;
    const Topology3D::SeamCollection& seams_;
    const DiscretizationResult3D& discretization3D_;
    const std::vector<size_t>& globalPointIndices_;
    std::function<size_t(size_t)> pointIdxToNode3DId_;
    const Geometry3D::GeometryCollection3D& geometry_;

    std::map<size_t, size_t> globalToLocal_;
    DiscretizationResult2D discretization2D_;
    std::vector<size_t> localIndexToNode3DId_;
};

} // namespace Meshing
