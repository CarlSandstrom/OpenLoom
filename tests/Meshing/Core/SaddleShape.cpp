#include "SaddleShape.h"

#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <Geom_BezierSurface.hxx>
#include <Precision.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Shell.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>

namespace TestSupport
{

namespace
{
constexpr double L = SADDLE_HALF_EXTENT;
constexpr double Z_BOTTOM = -5.0;
} // namespace

TopoDS_Shape buildSaddleSolid()
{
    const double squaredL = L * L;

    BRepBuilderAPI_Sewing sewing;

    // === Top face: saddle z = x²−y² (biquadratic Bezier, 3×3 control grid) ===
    // cᵢⱼ = αᵢ − βⱼ  where α = β = [L², −L², L²]
    {
        TColgp_Array2OfPnt poles(1, 3, 1, 3);
        poles.SetValue(1, 1, gp_Pnt(-L, -L, 0.0));
        poles.SetValue(1, 2, gp_Pnt(-L, 0.0, 2.0 * squaredL));
        poles.SetValue(1, 3, gp_Pnt(-L, L, 0.0));
        poles.SetValue(2, 1, gp_Pnt(0.0, -L, -2.0 * squaredL));
        poles.SetValue(2, 2, gp_Pnt(0.0, 0.0, 0.0));
        poles.SetValue(2, 3, gp_Pnt(0.0, L, -2.0 * squaredL));
        poles.SetValue(3, 1, gp_Pnt(L, -L, 0.0));
        poles.SetValue(3, 2, gp_Pnt(L, 0.0, 2.0 * squaredL));
        poles.SetValue(3, 3, gp_Pnt(L, L, 0.0));
        sewing.Add(BRepBuilderAPI_MakeFace(new Geom_BezierSurface(poles), Precision::Confusion()).Face());
    }

    // === Bottom face: flat rectangle at z = Z_BOTTOM ===
    {
        gp_Pln plane(gp_Pnt(0.0, 0.0, Z_BOTTOM), gp_Dir(0.0, 0.0, 1.0));
        sewing.Add(BRepBuilderAPI_MakeFace(plane, -L, L, -L, L).Face());
    }

    // === Left side (x = −L): ruled surface, top arc z = L²−y², bottom edge at z = Z_BOTTOM ===
    // Top row matches saddle column 1 (u=0) pole-for-pole.
    {
        TColgp_Array2OfPnt poles(1, 2, 1, 3);
        poles.SetValue(1, 1, gp_Pnt(-L, -L, 0.0));
        poles.SetValue(1, 2, gp_Pnt(-L, 0.0, 2.0 * squaredL));
        poles.SetValue(1, 3, gp_Pnt(-L, L, 0.0));
        poles.SetValue(2, 1, gp_Pnt(-L, -L, Z_BOTTOM));
        poles.SetValue(2, 2, gp_Pnt(-L, 0.0, Z_BOTTOM));
        poles.SetValue(2, 3, gp_Pnt(-L, L, Z_BOTTOM));
        sewing.Add(BRepBuilderAPI_MakeFace(new Geom_BezierSurface(poles), Precision::Confusion()).Face());
    }

    // === Right side (x = +L): ruled surface, top arc z = L²−y², bottom edge at z = Z_BOTTOM ===
    // Top row matches saddle column 3 (u=1) pole-for-pole.
    {
        TColgp_Array2OfPnt poles(1, 2, 1, 3);
        poles.SetValue(1, 1, gp_Pnt(L, -L, 0.0));
        poles.SetValue(1, 2, gp_Pnt(L, 0.0, 2.0 * squaredL));
        poles.SetValue(1, 3, gp_Pnt(L, L, 0.0));
        poles.SetValue(2, 1, gp_Pnt(L, -L, Z_BOTTOM));
        poles.SetValue(2, 2, gp_Pnt(L, 0.0, Z_BOTTOM));
        poles.SetValue(2, 3, gp_Pnt(L, L, Z_BOTTOM));
        sewing.Add(BRepBuilderAPI_MakeFace(new Geom_BezierSurface(poles), Precision::Confusion()).Face());
    }

    // === Front side (y = −L): ruled surface, top arc z = x²−L², bottom edge at z = Z_BOTTOM ===
    // Top row matches saddle row 1 (v=0) pole-for-pole.
    {
        TColgp_Array2OfPnt poles(1, 2, 1, 3);
        poles.SetValue(1, 1, gp_Pnt(-L, -L, 0.0));
        poles.SetValue(1, 2, gp_Pnt(0.0, -L, -2.0 * squaredL));
        poles.SetValue(1, 3, gp_Pnt(L, -L, 0.0));
        poles.SetValue(2, 1, gp_Pnt(-L, -L, Z_BOTTOM));
        poles.SetValue(2, 2, gp_Pnt(0.0, -L, Z_BOTTOM));
        poles.SetValue(2, 3, gp_Pnt(L, -L, Z_BOTTOM));
        sewing.Add(BRepBuilderAPI_MakeFace(new Geom_BezierSurface(poles), Precision::Confusion()).Face());
    }

    // === Back side (y = +L): ruled surface, top arc z = x²−L², bottom edge at z = Z_BOTTOM ===
    // Top row matches saddle row 3 (v=1) pole-for-pole.
    {
        TColgp_Array2OfPnt poles(1, 2, 1, 3);
        poles.SetValue(1, 1, gp_Pnt(-L, L, 0.0));
        poles.SetValue(1, 2, gp_Pnt(0.0, L, -2.0 * squaredL));
        poles.SetValue(1, 3, gp_Pnt(L, L, 0.0));
        poles.SetValue(2, 1, gp_Pnt(-L, L, Z_BOTTOM));
        poles.SetValue(2, 2, gp_Pnt(0.0, L, Z_BOTTOM));
        poles.SetValue(2, 3, gp_Pnt(L, L, Z_BOTTOM));
        sewing.Add(BRepBuilderAPI_MakeFace(new Geom_BezierSurface(poles), Precision::Confusion()).Face());
    }

    sewing.Perform();

    // Sewing only produces a closed TopoDS_Shell -- wrap it into an actual
    // TopoDS_Solid so downstream consumers that need genuine solid topology
    // (e.g. an inside/outside classifier querying the CAD model directly)
    // have one to query, not just a shell that happens to be watertight.
    const TopoDS_Shell shell = TopoDS::Shell(sewing.SewedShape());
    BRepBuilderAPI_MakeSolid solidMaker(shell);
    return solidMaker.Solid();
}

} // namespace TestSupport
