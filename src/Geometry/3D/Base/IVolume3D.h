#pragma once

#include "Common/Types.h"
#include <string>

namespace Geometry3D
{

/**
 * @brief Tri-state classification of a point relative to a volume's boundary
 *
 * Mirrors OpenCASCADE's TopAbs_State (IN/OUT/ON/UNKNOWN) without pulling an
 * OCC dependency into this OCC-agnostic interface. Kept as an exact
 * tri-state rather than collapsed to a bool: a point classifying as OnBoundary
 * is a meaningfully different answer from cleanly Inside or Outside -- e.g.
 * a point sitting exactly on the interface between two adjacent phases of a
 * multi-material domain -- and callers composing across several volumes
 * (see GeometryCollection3D::getVolume()) need that distinction to decide
 * which phase, if any, a point belongs to.
 */
enum class VolumeClassification
{
    Inside,
    Outside,
    OnBoundary,
    Unknown
};

/**
 * @brief Abstract interface for geometric volumes (solid bodies / material phases)
 *
 * Provides the point-classification query meshing needs to determine which
 * side of a boundary -- or which phase, in a multi-material domain -- a
 * given point belongs to, independent of any particular restricted-face
 * classification already computed for the mesh.
 */
class IVolume3D
{
public:
    virtual ~IVolume3D() = default;

    virtual std::string getId() const = 0;

    /// Classifies a 3D point relative to this volume's boundary.
    virtual VolumeClassification classifyPoint(const Meshing::Point3D& point) const = 0;
};

} // namespace Geometry3D
