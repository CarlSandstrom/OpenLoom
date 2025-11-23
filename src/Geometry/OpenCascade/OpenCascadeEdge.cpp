#include "OpenCascadeEdge.h"
#include <BRepAdaptor_Curve.hxx>
#include <BRep_Tool.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GeomLProp_CLProps.hxx>
#include <Precision.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <limits>
#include <sstream>

namespace Geometry
{

OpenCascadeEdge::OpenCascadeEdge(const TopoDS_Edge& edge) :
    edge_(edge)
{
}

Meshing::Point3D OpenCascadeEdge::getPoint(double t) const
{
    BRepAdaptor_Curve curve(edge_);
    gp_Pnt point = curve.Value(t);
    return Meshing::Point3D(point.X(), point.Y(), point.Z());
}

std::array<double, 3> OpenCascadeEdge::getTangent(double t) const
{
    BRepAdaptor_Curve curve(edge_);
    double tMin, tMax;
    Handle(Geom_Curve) geomCurve = BRep_Tool::Curve(edge_, tMin, tMax);

    if (!geomCurve.IsNull())
    {
        GeomLProp_CLProps props(geomCurve, t, 1, Precision::Confusion());

        if (props.IsTangentDefined())
        {
            gp_Dir tangentDir;
            props.Tangent(tangentDir);
            return {tangentDir.X(), tangentDir.Y(), tangentDir.Z()};
        }
    }

    // Fallback: compute tangent using finite differences
    gp_Pnt point;
    gp_Vec tangent;
    curve.D1(t, point, tangent);
    return {tangent.X(), tangent.Y(), tangent.Z()};
}

Meshing::Point3D OpenCascadeEdge::getStartPoint() const
{
    const auto [tMin, tMax] = getParameterBounds();
    return getPoint(tMin);
}

Meshing::Point3D OpenCascadeEdge::getEndPoint() const
{
    const auto [tMin, tMax] = getParameterBounds();
    return getPoint(tMax);
}

std::pair<double, double> OpenCascadeEdge::getParameterBounds() const
{
    BRepAdaptor_Curve curve(edge_);
    return {curve.FirstParameter(), curve.LastParameter()};
}

double OpenCascadeEdge::getLength() const
{
    BRepAdaptor_Curve curve(edge_);
    const auto [tMin, tMax] = getParameterBounds();

    // Use GCPnts_AbscissaPoint to compute the actual arc length
    double length = GCPnts_AbscissaPoint::Length(curve, tMin, tMax);

    if (length > Precision::Confusion())
    {
        return length;
    }

    // Fallback: return parameter range (not ideal but better than 0)
    return tMax - tMin;
}

double OpenCascadeEdge::getCurvature(double t) const
{
    BRepAdaptor_Curve curve(edge_);
    double tMin, tMax;
    Handle(Geom_Curve) geomCurve = BRep_Tool::Curve(edge_, tMin, tMax);

    if (!geomCurve.IsNull())
    {
        GeomLProp_CLProps props(geomCurve, t, 2, Precision::Confusion());

        if (props.IsTangentDefined())
        {
            return props.Curvature();
        }
    }

    // Fallback: return 0 if curvature cannot be computed
    return 0.0;
}

std::string OpenCascadeEdge::getId() const
{
    std::ostringstream oss;
    oss << "OpenCascadeEdge_" << std::hex << reinterpret_cast<uintptr_t>(&edge_);
    return oss.str();
}

} // namespace Geometry