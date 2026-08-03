#include "OpenCascadeSurface.h"
#include <Bnd_Box.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepClass_FaceClassifier.hxx>
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
#include <gp_Vec.hxx>
#include <limits>
#include <sstream>

namespace Geometry3D
{

OpenCascadeSurface::OpenCascadeSurface(const TopoDS_Face& face) :
    face_(face)
{
}

std::array<double, 3> OpenCascadeSurface::getNormal(double u, double v) const
{
    BRepAdaptor_Surface surface(face_);
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

Meshing::Point3D OpenCascadeSurface::getPoint(double u, double v) const
{
    BRepAdaptor_Surface surface(face_);
    gp_Pnt point = surface.Value(u, v);
    return Meshing::Point3D(point.X(), point.Y(), point.Z());
}

Common::BoundingBox2D OpenCascadeSurface::getParameterBounds() const
{
    BRepAdaptor_Surface surface(face_);
    return Common::BoundingBox2D(surface.FirstUParameter(), surface.LastUParameter(),
                                 surface.FirstVParameter(), surface.LastVParameter());
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
    {
        return std::nullopt;
    }

    ShapeAnalysis_Surface analyzer(geomSurface);
    gp_Pnt2d uv = analyzer.ValueOfUV(gp_Pnt(point.x(), point.y(), point.z()),
                                      Precision::Confusion());
    return Meshing::Point2D(uv.X(), uv.Y());
}

std::optional<Meshing::Point2D> OpenCascadeSurface::projectPointToUnderlyingSurface(
    const Meshing::Point3D& point,
    const Meshing::Point2D& seedUV) const
{
    Handle(Geom_Surface) geomSurface = BRep_Tool::Surface(face_);
    if (geomSurface.IsNull())
    {
        return std::nullopt;
    }

    ShapeAnalysis_Surface analyzer(geomSurface);
    gp_Pnt2d seed(seedUV.x(), seedUV.y());
    gp_Pnt2d uv = analyzer.NextValueOfUV(seed,
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

} // namespace Geometry3D