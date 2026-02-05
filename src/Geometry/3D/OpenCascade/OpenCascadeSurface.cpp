#include "OpenCascadeSurface.h"
#include <BRepAdaptor_Surface.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GeomLProp_SLProps.hxx>
#include <Precision.hxx>
#include <ShapeAnalysis.hxx>
#include <TopoDS_Shape.hxx>
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

std::string OpenCascadeSurface::getId() const
{
    std::ostringstream oss;
    oss << "OpenCascadeSurface_" << std::hex << std::hash<TopoDS_Shape>{}(face_);
    return oss.str();
}

} // namespace Geometry3D