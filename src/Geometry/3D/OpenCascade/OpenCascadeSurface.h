#pragma once

#include "../Base/ISurface3D.h"
#include <TopoDS_Face.hxx>
#include <memory>

// Forward declare to avoid pulling OCC headers into translation units that
// only need the ISurface3D interface. Both are plain classes so forward
// declarations are valid.
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
    // Destructor must be defined in the .cpp where ShapeAnalysis_Surface is
    // complete (required by std::unique_ptr<ShapeAnalysis_Surface>).
    ~OpenCascadeSurface() override;

    std::array<double, 3> getNormal(double u, double v) const override;
    Meshing::Point3D getPoint(double u, double v) const override;
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
    // Both adaptors are lazily initialized on first use and reused for all
    // subsequent calls. BRepAdaptor_Surface construction is non-trivial (it
    // wraps the face's geometric surface, resolves orientation, etc.) and
    // creating a new one on every getPoint()/getNormal() call adds meaningful
    // overhead when the surface is evaluated thousands of times during
    // tessellation build and classification. ShapeAnalysis_Surface::ValueOfUV
    // additionally builds an internal Extrema grid (O(Nu × Nv) evaluations)
    // on first use and caches it — re-creating it every call would rebuild
    // that grid each time, which is catastrophic for RCDT refinement on Bezier
    // surfaces.
    // The UV classifier is cached for the same reason as the two adaptors
    // above, and it matters more than either. A BRepClass_FaceClassifier is
    // built from scratch on construction -- it walks the face's wires, loads
    // every pcurve and prepares intersectors -- so classifying one point that
    // way pays for the whole face each time. Profiling the cylinder example
    // (callgrind, Release) put isUVWithinTrimmedBoundary at 78% of total
    // runtime, most of it inside that repeated setup and the allocator churn
    // it produces. BRepTopAdaptor_FClass2d exists precisely to be built once
    // per face and asked many times.
    TopoDS_Face face_;
    mutable std::unique_ptr<BRepAdaptor_Surface> surfaceAdaptor_;
    mutable std::unique_ptr<ShapeAnalysis_Surface> surfaceAnalyzer_;
    mutable std::unique_ptr<BRepTopAdaptor_FClass2d> uvClassifier_;
};

} // namespace Geometry3D