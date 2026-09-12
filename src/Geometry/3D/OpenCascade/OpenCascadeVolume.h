#pragma once

#include "../Base/IVolume3D.h"
#include <TopoDS_Solid.hxx>
#include <memory>

// Forward declare to avoid pulling OCC headers into translation units that
// only need the IVolume3D interface. Plain class, so a forward declaration
// is valid -- see OpenCascadeSurface's adaptors for the same pattern.
class BRepClass3d_SolidClassifier;

namespace Geometry3D
{

/**
 * @brief OpenCASCADE implementation of Volume (a solid body / material phase)
 */
class OpenCascadeVolume : public IVolume3D
{
public:
    explicit OpenCascadeVolume(const TopoDS_Solid& solid);
    // Destructor must be defined in the .cpp where BRepClass3d_SolidClassifier
    // is complete (required by std::unique_ptr<BRepClass3d_SolidClassifier>).
    ~OpenCascadeVolume() override;

    std::string getId() const override;
    VolumeClassification classifyPoint(const Meshing::Point3D& point) const override;

private:
    TopoDS_Solid solid_;

    // Lazily initialized on first use and reused for all subsequent calls:
    // constructing BRepClass3d_SolidClassifier builds an acceleration
    // structure over the solid's faces, which would be wasteful to rebuild
    // on every classifyPoint() call -- see OpenCascadeSurface's adaptors for
    // the same rationale.
    mutable std::unique_ptr<BRepClass3d_SolidClassifier> classifier_;

    // Cached on first use -- see OpenCascadeSurface::isPointWithinTrimmedBoundary
    // for why classification tolerance needs to scale with the geometry's own
    // size rather than a bare Precision::Confusion(): a query point (a tet
    // circumcenter) has typically gone through RCDT's own accumulated
    // arithmetic, so a point genuinely meant to sit on the boundary can be
    // numerically fuzzy well past kernel-level tolerance.
    mutable double diameter_ = -1.0;
};

} // namespace Geometry3D
