#pragma once

#include "Common/Types.h"

#include <string>
#include <vector>

namespace Geometry3D
{
class GeometryCollection3D;
} // namespace Geometry3D

namespace Meshing
{

/// Which side of the model a point falls on, as resolved against every
/// IVolume3D in the geometry.
enum class PointPhaseKind
{
    Exterior,
    InVolume,

    /// No answer, rather than a third side. Reached when the CAD kernel
    /// reports the point as lying ON a boundary (inside its classification
    /// tolerance band -- max(Precision::Confusion(), 1e-4 * diameter), which
    /// is ~1e-3 on a unit-scale model) or cannot decide, or when the point
    /// classifies as inside two different volumes at once.
    ///
    /// This is load-bearing and must not be collapsed into a guess: the band
    /// is real geometry at the 1e-3 scale, not floating-point noise, and
    /// OPE-176 measured confidently-wrong answers to be more damaging than
    /// honest refusals. A flat boundary sliver's centroid lands here
    /// routinely -- see OPE-187.
    Ambiguous
};

struct PointPhase
{
    PointPhaseKind kind = PointPhaseKind::Exterior;
    std::string volumeId;
};

/// Resolves a point against every volume in the geometry.
///
/// Shared rather than file-local so that the restriction oracle and the phase
/// diagnostics answer from the identical function: a diagnostic that
/// reimplemented this could drift from what the mesher actually does, which
/// would make the picture it draws quietly wrong.
PointPhase classifyPointPhase(const Point3D& point,
                              const std::vector<std::string>& volumeIds,
                              const Geometry3D::GeometryCollection3D& geometry);

} // namespace Meshing
