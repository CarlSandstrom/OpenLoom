#pragma once

#include "Geometry/3D/Base/ISurface3D.h"
#include <TopoDS_Face.hxx>
#include <memory>

// Forward declared to keep OpenCASCADE headers out of translation units that
// only need the ISurface3D interface.
class BRepAdaptor_Surface;
class BRepTopAdaptor_FClass2d;
class ShapeAnalysis_Surface;

namespace Geometry3D
{

/**
 * @brief OpenCASCADE implementation of Surface
 */
class OpenCascadeSurface : public ISurface3D
{
public:
    explicit OpenCascadeSurface(const TopoDS_Face& face);
    // Defined in the .cpp, where the cached OpenCASCADE types are complete.
    ~OpenCascadeSurface() override;

    Meshing::Vector3D getNormal(double u, double v) const override;
    Meshing::Point3D getPoint(double u, double v) const override;
    SurfaceDerivatives getFirstDerivatives(double u, double v) const override;
    Common::BoundingBox2D getParameterBounds() const override;
    double getGap(const Meshing::Point3D& point) const override;
    Meshing::Point2D projectPoint(const Meshing::Point3D& point) const override;
    std::optional<Meshing::Point2D> projectPointToUnderlyingSurface(
        const Meshing::Point3D& point) const override;
    std::optional<Meshing::Point2D> projectPointToUnderlyingSurface(
        const Meshing::Point3D& point,
        const Meshing::Point2D& seedUV) const override;
    std::string getId() const override;
    std::optional<PrincipalCurvatures> getPrincipalCurvatures(double u, double v) const override;
    bool isPointWithinTrimmedBoundary(const Meshing::Point3D& point) const override;
    bool isUVWithinTrimmedBoundary(double u, double v) const override;

private:
    // The adaptor, analyzer and classifier are built lazily on first use and
    // reused. Each is expensive to construct and the surface is queried
    // thousands of times during tessellation and classification:
    // ShapeAnalysis_Surface builds an internal extrema grid on first use, and
    // BRepTopAdaptor_FClass2d walks every wire and pcurve of the face, so
    // rebuilding either per call dominates the runtime.
    TopoDS_Face face_;
    mutable std::unique_ptr<BRepAdaptor_Surface> surfaceAdaptor_;
    mutable std::unique_ptr<ShapeAnalysis_Surface> surfaceAnalyzer_;
    mutable std::unique_ptr<BRepTopAdaptor_FClass2d> uvClassifier_;
};

} // namespace Geometry3D