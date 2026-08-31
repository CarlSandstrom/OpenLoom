#include "OpenCascadeSurface.h"
#include <Bnd_Box.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepTopAdaptor_FClass2d.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GeomLProp_SLProps.hxx>
#include <Precision.hxx>
#include <ShapeAnalysis.hxx>
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

namespace Geometry3D
{

OpenCascadeSurface::OpenCascadeSurface(const TopoDS_Face& face) :
    face_(face)
{
}

// Defined here so that the compiler sees the complete type of
// ShapeAnalysis_Surface when instantiating std::unique_ptr's destructor.
OpenCascadeSurface::~OpenCascadeSurface() = default;

std::array<double, 3> OpenCascadeSurface::getNormal(double u, double v) const
{
    Handle(Geom_Surface) geomSurface = BRep_Tool::Surface(face_);
    GeomLProp_SLProps props(geomSurface, u, v, 1, Precision::Confusion());

    if (props.IsNormalDefined())
    {
        gp_Vec normal = props.Normal();

        // Flip normal for reversed face orientation
        if (face_.Orientation() == TopAbs_REVERSED)
        {
            normal.Reverse();
        }

        return {normal.X(), normal.Y(), normal.Z()};
    }

    // Fallback: return a default normal if not defined
    return {0.0, 0.0, 1.0};
}

std::optional<PrincipalCurvatures> OpenCascadeSurface::getPrincipalCurvatures(double u,
                                                                              double v) const
{
    Handle(Geom_Surface) geomSurface = BRep_Tool::Surface(face_);
    GeomLProp_SLProps props(geomSurface, u, v, 2, Precision::Confusion());

    if (!props.IsCurvatureDefined())
    {
        return std::nullopt;
    }

    double minimum = props.MinCurvature();
    double maximum = props.MaxCurvature();

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
    if (!surfaceAdaptor_)
        surfaceAdaptor_ = std::make_unique<BRepAdaptor_Surface>(face_);
    gp_Pnt point = surfaceAdaptor_->Value(u, v);
    return Meshing::Point3D(point.X(), point.Y(), point.Z());
}

Common::BoundingBox2D OpenCascadeSurface::getParameterBounds() const
{
    if (!surfaceAdaptor_)
        surfaceAdaptor_ = std::make_unique<BRepAdaptor_Surface>(face_);
    return Common::BoundingBox2D(surfaceAdaptor_->FirstUParameter(), surfaceAdaptor_->LastUParameter(),
                                 surfaceAdaptor_->FirstVParameter(), surfaceAdaptor_->LastVParameter());
}

double OpenCascadeSurface::getGap(const Meshing::Point3D& point) const
{
    Handle(Geom_Surface) geomSurface = BRep_Tool::Surface(face_);

    gp_Pnt queryPoint(point.x(), point.y(), point.z());
    GeomAPI_ProjectPointOnSurf projector(queryPoint, geomSurface);

    if (projector.NbPoints() > 0)
    {
        return projector.LowerDistance();
    }

    // Return a large value if projection fails
    return std::numeric_limits<double>::max();
}

Meshing::Point2D OpenCascadeSurface::projectPoint(const Meshing::Point3D& point) const
{
    Handle(Geom_Surface) geomSurface = BRep_Tool::Surface(face_);

    gp_Pnt queryPoint(point.x(), point.y(), point.z());
    GeomAPI_ProjectPointOnSurf projector(queryPoint, geomSurface);

    if (projector.NbPoints() > 0)
    {
        double u, v;
        projector.LowerDistanceParameters(u, v);
        return Meshing::Point2D(u, v);
    }

    // Return parameter bounds center if projection fails
    const Common::BoundingBox2D bounds = getParameterBounds();
    return Meshing::Point2D((bounds.getUMin() + bounds.getUMax()) / 2.0,
                            (bounds.getVMin() + bounds.getVMax()) / 2.0);
}

std::optional<Meshing::Point2D> OpenCascadeSurface::projectPointToUnderlyingSurface(
    const Meshing::Point3D& point) const
{
    Handle(Geom_Surface) geomSurface = BRep_Tool::Surface(face_);
    if (geomSurface.IsNull())
        return std::nullopt;

    if (!surfaceAnalyzer_)
        surfaceAnalyzer_ = std::make_unique<ShapeAnalysis_Surface>(geomSurface);

    gp_Pnt2d uv = surfaceAnalyzer_->ValueOfUV(gp_Pnt(point.x(), point.y(), point.z()),
                                               Precision::Confusion());
    return Meshing::Point2D(uv.X(), uv.Y());
}

std::optional<Meshing::Point2D> OpenCascadeSurface::projectPointToUnderlyingSurface(
    const Meshing::Point3D& point,
    const Meshing::Point2D& seedUV) const
{
    Handle(Geom_Surface) geomSurface = BRep_Tool::Surface(face_);
    if (geomSurface.IsNull())
        return std::nullopt;

    if (!surfaceAnalyzer_)
        surfaceAnalyzer_ = std::make_unique<ShapeAnalysis_Surface>(geomSurface);

    gp_Pnt2d seed(seedUV.x(), seedUV.y());
    gp_Pnt2d uv = surfaceAnalyzer_->NextValueOfUV(seed,
                                                   gp_Pnt(point.x(), point.y(), point.z()),
                                                   Precision::Confusion());
    return Meshing::Point2D(uv.X(), uv.Y());
}

std::string OpenCascadeSurface::getId() const
{
    std::ostringstream oss;
    oss << "OpenCascadeSurface_" << std::hex << std::hash<TopoDS_Shape>{}(face_);
    return oss.str();
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
    const double tolerance = std::max(Precision::Confusion(), RELATIVE_TOLERANCE * diameter);

    BRepClass_FaceClassifier classifier(face_, gp_Pnt(point.x(), point.y(), point.z()), tolerance);
    const TopAbs_State state = classifier.State();
    return state == TopAbs_IN || state == TopAbs_ON;
}

bool OpenCascadeSurface::isUVWithinTrimmedBoundary(double u, double v) const
{
    // Classify the UV point directly in parameter space. This avoids the
    // Extrema_GenExtPS surface-projection step that the 3D
    // BRepClass_FaceClassifier overload triggers for non-analytic surfaces
    // (Bezier, B-spline), which is O(extrema_grid²) per point; classifying in
    // UV works purely against the face's PCurves and needs no projection.
    //
    // The classifier is built once and reused (see the member's own comment).
    // Constructing a BRepClass_FaceClassifier per call instead re-walked the
    // face's wires and reloaded its pcurves for every single point.
    if (!uvClassifier_)
        uvClassifier_ = std::make_unique<BRepTopAdaptor_FClass2d>(face_, Precision::PConfusion());

    const TopAbs_State state = uvClassifier_->Perform(gp_Pnt2d(u, v));
    return state == TopAbs_IN || state == TopAbs_ON;
}

} // namespace Geometry3D