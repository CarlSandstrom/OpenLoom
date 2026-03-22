#pragma once

#include "PeriodicDomain2D.h"
#include "PeriodicOffsetTable.h"
#include "Common/Types.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include <tuple>

namespace Meshing
{

/// Wraps MeshData2D to provide offset-aware coordinate lookup for periodic
/// 2D triangulations.
///
/// In a periodic triangulation some triangles straddle a period boundary.
/// Their vertices are stored at canonical UV coordinates in [0, period) inside
/// MeshData2D, but geometric predicates (circumcircle tests, encroachment
/// checks, etc.) must operate on effective coordinates that are shifted by the
/// per-triangle offset stored in PeriodicOffsetTable.
///
/// This class is the single point of truth for "what coordinates do I use for
/// this vertex in this triangle?".  Non-periodic triangulations never
/// instantiate it; they read MeshData2D directly.
class PeriodicMeshData2D
{
public:
    PeriodicMeshData2D(const MeshData2D& meshData, const PeriodicDomainConfig& config);

    const PeriodicDomainConfig& getConfig() const { return config_; }

    PeriodicOffsetTable& getOffsetTable() { return offsetTable_; }
    const PeriodicOffsetTable& getOffsetTable() const { return offsetTable_; }

    /// Returns the effective (offset-shifted) coordinate of the vertex at
    /// slot [0, 1, 2] of the triangle with the given ID.
    Point2D getCoordinate(size_t triangleId, int slot) const;

    /// Returns all three effective coordinates of a triangle.
    std::tuple<Point2D, Point2D, Point2D> getTriangleCoordinates(size_t triangleId) const;

    /// Returns the nearest periodic copy of 'point' relative to 'referencePoint'.
    ///
    /// Used when testing whether an insertion candidate falls inside a
    /// triangle's circumcircle: the candidate may need to be shifted into the
    /// same local frame as the triangle before the circle test.
    Point2D nearestCopy(const Point2D& point, const Point2D& referencePoint) const;

    /// Applies the given offset shift to a canonical coordinate.
    Point2D applyOffset(const Point2D& canonicalCoord, const PeriodicOffset& offset) const;

private:
    const MeshData2D& meshData_;
    PeriodicDomainConfig config_;
    PeriodicOffsetTable offsetTable_;
};

} // namespace Meshing
