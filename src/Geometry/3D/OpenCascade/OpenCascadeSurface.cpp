#include "OpenCascadeSurface.h"
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepTopAdaptor_FClass2d.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GeomLProp_SLProps.hxx>
#include <Precision.hxx>
#include <ShapeAnalysis_Surface.hxx>
#include <TopAbs_State.hxx>
#include <TopoDS_Shape.hxx>
#include <algorithm>
#include <cmath>
#include <functional>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec.hxx>
#include <limits>
#include <sstream>
#include <utility>

namespace
{
template <typename T, typename... Arguments>
T& getOrCreate(std::unique_ptr<T>& cache, Arguments&&... arguments);
gp_Pnt toOpenCascadePoint(const Meshing::Point3D& point);
} // namespace

namespace Geometry3D
{

OpenCascadeSurface::OpenCascadeSurface(const TopoDS_Face& face) :
    face_(face)
{
}

// Defined here, where the cached OpenCASCADE types are complete, as
// std::unique_ptr's destructor requires.
OpenCascadeSurface::~OpenCascadeSurface() = default;

Meshing::Vector3D OpenCascadeSurface::getNormal(double u, double v) const
{
    GeomLProp_SLProps properties(BRep_Tool::Surface(face_), u, v, 1, Precision::Confusion());

    if (properties.IsNormalDefined())
    {
        gp_Vec normal = properties.Normal();
        if (face_.Orientation() == TopAbs_REVERSED) normal.Reverse();
        return {normal.X(), normal.Y(), normal.Z()};
    }

    return {0.0, 0.0, 1.0};
}

std::optional<PrincipalCurvatures> OpenCascadeSurface::getPrincipalCurvatures(double u,
                                                                              double v) const
{
    GeomLProp_SLProps properties(BRep_Tool::Surface(face_), u, v, 2, Precision::Confusion());

    if (!properties.IsCurvatureDefined()) return std::nullopt;

    double minimum = properties.MinCurvature();
    double maximum = properties.MaxCurvature();

    // OCC signs both curvatures against the underlying geometric surface's
    // own normal. getNormal() reverses that normal for a REVERSED face, so
    // the signs must be reversed here too or the two would disagree about
    // which way the surface bends. Negating swaps the ordering as well,
    // since -maximum <= -minimum.
    if (face_.Orientation() == TopAbs_REVERSED)
    {
        return PrincipalCurvatures{-maximum, -minimum};
    }

    return PrincipalCurvatures{minimum, maximum};
}

Meshing::Point3D OpenCascadeSurface::getPoint(double u, double v) const
{
    gp_Pnt point = getOrCreate(surfaceAdaptor_, face_).Value(u, v);
    return Meshing::Point3D(point.X(), point.Y(), point.Z());
}

SurfaceDerivatives OpenCascadeSurface::getFirstDerivatives(double u, double v) const
{
    gp_Pnt point;
    gp_Vec derivativeU;
    gp_Vec derivativeV;
    getOrCreate(surfaceAdaptor_, face_).D1(u, v, point, derivativeU, derivativeV);
    return {{point.X(), point.Y(), point.Z()},
            {derivativeU.X(), derivativeU.Y(), derivativeU.Z()},
            {derivativeV.X(), derivativeV.Y(), derivativeV.Z()}};
}

Common::BoundingBox2D OpenCascadeSurface::getParameterBounds() const
{
    auto& adaptor = getOrCreate(surfaceAdaptor_, face_);
    return Common::BoundingBox2D(adaptor.FirstUParameter(), adaptor.LastUParameter(),
                                 adaptor.FirstVParameter(), adaptor.LastVParameter());
}

double OpenCascadeSurface::getGap(const Meshing::Point3D& point) const
{
    GeomAPI_ProjectPointOnSurf projector(toOpenCascadePoint(point), BRep_Tool::Surface(face_));

    if (projector.NbPoints() > 0) return projector.LowerDistance();
    return std::numeric_limits<double>::max();
}

Meshing::Point2D OpenCascadeSurface::projectPoint(const Meshing::Point3D& point) const
{
    GeomAPI_ProjectPointOnSurf projector(toOpenCascadePoint(point), BRep_Tool::Surface(face_));

    if (projector.NbPoints() > 0)
    {
        double u;
        double v;
        projector.LowerDistanceParameters(u, v);
        return Meshing::Point2D(u, v);
    }

    auto bounds = getParameterBounds();
    return Meshing::Point2D((bounds.getUMin() + bounds.getUMax()) / 2.0,
                            (bounds.getVMin() + bounds.getVMax()) / 2.0);
}

std::optional<Meshing::Point2D> OpenCascadeSurface::projectPointToUnderlyingSurface(
    const Meshing::Point3D& point) const
{
    Handle(Geom_Surface) geometricSurface = BRep_Tool::Surface(face_);
    if (geometricSurface.IsNull())
        return std::nullopt;

    gp_Pnt2d uv = getOrCreate(surfaceAnalyzer_, geometricSurface).ValueOfUV(toOpenCascadePoint(point), Precision::Confusion());
    return Meshing::Point2D(uv.X(), uv.Y());
}

std::optional<Meshing::Point2D> OpenCascadeSurface::projectPointToUnderlyingSurface(
    const Meshing::Point3D& point,
    const Meshing::Point2D& seedUV) const
{
    Handle(Geom_Surface) geometricSurface = BRep_Tool::Surface(face_);
    if (geometricSurface.IsNull())
        return std::nullopt;

    gp_Pnt2d seed(seedUV.x(), seedUV.y());
    gp_Pnt2d uv = getOrCreate(surfaceAnalyzer_, geometricSurface).NextValueOfUV(seed, toOpenCascadePoint(point), Precision::Confusion());
    return Meshing::Point2D(uv.X(), uv.Y());
}

std::string OpenCascadeSurface::getId() const
{
    std::ostringstream stream;
    stream << "OpenCascadeSurface_" << std::hex << std::hash<TopoDS_Shape>{}(face_);
    return stream.str();
}

bool OpenCascadeSurface::isPointWithinTrimmedBoundary(const Meshing::Point3D& point) const
{
    // Precision::Confusion() (1e-7) alone is too tight here: node coordinates
    // reaching this point have gone through the RCDT pipeline's own
    // arithmetic (circumcenter computation, bisection), which accumulates
    // floating-point drift well past a geometric-kernel-level tolerance for
    // a point that is genuinely meant to sit on this face. Scale the
    // tolerance to the face's own size so a point that's merely numerically
    // fuzzy still classifies correctly, while a point genuinely past the
    // trim boundary (the actual crease case this method exists for) is
    // still rejected.
    Bnd_Box box;
    BRepBndLib::Add(face_, box);
    double diameter = box.IsVoid() ? 0.0 : std::sqrt(box.SquareExtent());
    if (diameter <= 0.0)
        diameter = 1.0;

    constexpr double RELATIVE_TOLERANCE = 1e-4;
    auto tolerance = std::max(Precision::Confusion(), RELATIVE_TOLERANCE * diameter);

    BRepClass_FaceClassifier classifier(face_, toOpenCascadePoint(point), tolerance);
    auto state = classifier.State();
    return state == TopAbs_IN || state == TopAbs_ON;
}

bool OpenCascadeSurface::isUVWithinTrimmedBoundary(double u, double v) const
{
    // Classify the UV point directly in parameter space. This avoids the
    // Extrema_GenExtPS surface-projection step that the 3D
    // BRepClass_FaceClassifier overload triggers for non-analytic surfaces
    // (Bezier, B-spline), which is O(extrema_grid²) per point; classifying in
    // UV works purely against the face's PCurves and needs no projection.
    auto state = getOrCreate(uvClassifier_, face_, Precision::PConfusion()).Perform(gp_Pnt2d(u, v));
    return state == TopAbs_IN || state == TopAbs_ON;
}

} // namespace Geometry3D

namespace
{

template <typename T, typename... Arguments>
T& getOrCreate(std::unique_ptr<T>& cache, Arguments&&... arguments)
{
    if (!cache) cache = std::make_unique<T>(std::forward<Arguments>(arguments)...);
    return *cache;
}

gp_Pnt toOpenCascadePoint(const Meshing::Point3D& point)
{
    return gp_Pnt(point.x(), point.y(), point.z());
}

} // namespace