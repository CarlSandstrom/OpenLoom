#include "OpenCascadeEdge.h"
#include <BRepAdaptor_Curve.hxx>
#include <BRep_Tool.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GeomLProp_CLProps.hxx>
#include <Precision.hxx>
#include <TopoDS_Shape.hxx>
#include <functional>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <sstream>

namespace Geometry3D
{

OpenCascadeEdge::OpenCascadeEdge(const TopoDS_Edge& edge) :
    edge_(edge)
{
}

Meshing::Point3D OpenCascadeEdge::getPoint(double parameter) const
{
    BRepAdaptor_Curve curve(edge_);
    gp_Pnt point = curve.Value(parameter);
    return Meshing::Point3D(point.X(), point.Y(), point.Z());
}

Meshing::Vector3D OpenCascadeEdge::getTangent(double parameter) const
{
    BRepAdaptor_Curve curve(edge_);
    double parameterMin, parameterMax;
    Handle(Geom_Curve) geometricCurve = BRep_Tool::Curve(edge_, parameterMin, parameterMax);

    if (!geometricCurve.IsNull())
    {
        GeomLProp_CLProps properties(geometricCurve, parameter, 1, Precision::Confusion());

        if (properties.IsTangentDefined())
        {
            gp_Dir tangentDirection;
            properties.Tangent(tangentDirection);
            return {tangentDirection.X(), tangentDirection.Y(), tangentDirection.Z()};
        }
    }

    // Fallback for a curve without a defined tangent: the first derivative
    gp_Pnt point;
    gp_Vec tangent;
    curve.D1(parameter, point, tangent);
    return {tangent.X(), tangent.Y(), tangent.Z()};
}

Meshing::Point3D OpenCascadeEdge::getStartPoint() const
{
    auto [parameterMin, parameterMax] = getParameterBounds();
    return getPoint(parameterMin);
}

Meshing::Point3D OpenCascadeEdge::getEndPoint() const
{
    auto [parameterMin, parameterMax] = getParameterBounds();
    return getPoint(parameterMax);
}

std::pair<double, double> OpenCascadeEdge::getParameterBounds() const
{
    BRepAdaptor_Curve curve(edge_);
    return {curve.FirstParameter(), curve.LastParameter()};
}

double OpenCascadeEdge::getLength() const
{
    BRepAdaptor_Curve curve(edge_);
    auto [parameterMin, parameterMax] = getParameterBounds();

    double length = GCPnts_AbscissaPoint::Length(curve, parameterMin, parameterMax);

    if (length > Precision::Confusion())
    {
        return length;
    }

    // A degenerate arc length falls back to the parameter range rather than 0
    return parameterMax - parameterMin;
}

double OpenCascadeEdge::getParameterAtArcLengthFraction(double parameterStart, double parameterEnd, double fraction) const
{
    BRepAdaptor_Curve curve(edge_);
    double totalLength = GCPnts_AbscissaPoint::Length(curve, parameterStart, parameterEnd);

    if (totalLength <= Precision::Confusion())
    {
        return parameterStart + fraction * (parameterEnd - parameterStart);
    }

    GCPnts_AbscissaPoint solver(curve, fraction * totalLength, parameterStart);
    return solver.Parameter();
}

double OpenCascadeEdge::getCurvature(double parameter) const
{
    double parameterMin, parameterMax;
    Handle(Geom_Curve) geometricCurve = BRep_Tool::Curve(edge_, parameterMin, parameterMax);

    if (!geometricCurve.IsNull())
    {
        GeomLProp_CLProps properties(geometricCurve, parameter, 2, Precision::Confusion());

        if (properties.IsTangentDefined())
        {
            return properties.Curvature();
        }
    }

    return 0.0;
}

bool OpenCascadeEdge::isDegenerate() const
{
    return BRep_Tool::Degenerated(edge_);
}

std::string OpenCascadeEdge::getId() const
{
    std::ostringstream oss;
    oss << "OpenCascadeEdge_" << std::hex << std::hash<TopoDS_Shape>{}(edge_);
    return oss.str();
}

} // namespace Geometry3D