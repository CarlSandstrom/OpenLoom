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
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>

#include <array>

// Mirrors buildSaddleSolid() in src/Examples/3D/Surface/SaddleSurfaceMesh.cc
// function for function; keep the two in step (see SaddleShape.h).

namespace TestSupport
{

namespace
{

constexpr double LENGTH = SADDLE_HALF_EXTENT;
constexpr double Z_BOTTOM = -5.0; // flat bottom, below the lowest saddle point (z = -L² = -4)

// Control grid of the saddle z = x²−y² as a biquadratic Bezier surface.
// Index i runs along x and j along y; cᵢⱼ = αᵢ − βⱼ with α = β = [L², −L², L²].
TColgp_Array2OfPnt makeSaddlePoles()
{
    double squaredLength = LENGTH * LENGTH;

    TColgp_Array2OfPnt poles(1, 3, 1, 3);
    poles.SetValue(1, 1, gp_Pnt(-LENGTH, -LENGTH, 0.0));
    poles.SetValue(1, 2, gp_Pnt(-LENGTH, 0.0, 2.0 * squaredLength));
    poles.SetValue(1, 3, gp_Pnt(-LENGTH, LENGTH, 0.0));
    poles.SetValue(2, 1, gp_Pnt(0.0, -LENGTH, -2.0 * squaredLength));
    poles.SetValue(2, 2, gp_Pnt(0.0, 0.0, 0.0));
    poles.SetValue(2, 3, gp_Pnt(0.0, LENGTH, -2.0 * squaredLength));
    poles.SetValue(3, 1, gp_Pnt(LENGTH, -LENGTH, 0.0));
    poles.SetValue(3, 2, gp_Pnt(LENGTH, 0.0, 2.0 * squaredLength));
    poles.SetValue(3, 3, gp_Pnt(LENGTH, LENGTH, 0.0));
    return poles;
}

TopoDS_Face makeBezierFace(const TColgp_Array2OfPnt& poles)
{
    return BRepBuilderAPI_MakeFace(new Geom_BezierSurface(poles), Precision::Confusion()).Face();
}

TopoDS_Face makeBottomFace()
{
    gp_Pln plane(gp_Pnt(0.0, 0.0, Z_BOTTOM), gp_Dir(0.0, 0.0, 1.0));
    return BRepBuilderAPI_MakeFace(plane, -LENGTH, LENGTH, -LENGTH, LENGTH).Face();
}

// A side face: a Bezier surface linear in U (depth) and quadratic in V (along
// the arc). Its top row is the saddle's boundary arc pole-for-pole, so the
// shared edge is geometrically identical and sewing closes it without a gap;
// its bottom row is the same arc dropped to z = Z_BOTTOM.
TopoDS_Face makeRuledSideFace(const std::array<gp_Pnt, 3>& arcPoles)
{
    TColgp_Array2OfPnt poles(1, 2, 1, 3);
    for (int column = 1; column <= 3; ++column)
    {
        auto& arcPole = arcPoles[column - 1];
        poles.SetValue(1, column, arcPole);
        poles.SetValue(2, column, gp_Pnt(arcPole.X(), arcPole.Y(), Z_BOTTOM));
    }
    return makeBezierFace(poles);
}

} // namespace

TopoDS_Shape buildSaddleSolid()
{
    auto saddlePoles = makeSaddlePoles();

    BRepBuilderAPI_Sewing sewing;
    sewing.Add(makeBezierFace(saddlePoles));
    sewing.Add(makeBottomFace());
    sewing.Add(makeRuledSideFace({saddlePoles(1, 1), saddlePoles(1, 2), saddlePoles(1, 3)})); // x = −L
    sewing.Add(makeRuledSideFace({saddlePoles(3, 1), saddlePoles(3, 2), saddlePoles(3, 3)})); // x = +L
    sewing.Add(makeRuledSideFace({saddlePoles(1, 1), saddlePoles(2, 1), saddlePoles(3, 1)})); // y = −L
    sewing.Add(makeRuledSideFace({saddlePoles(1, 3), saddlePoles(2, 3), saddlePoles(3, 3)})); // y = +L
    sewing.Perform();

    // Sewing only produces a closed TopoDS_Shell -- wrap it into an actual
    // TopoDS_Solid so downstream consumers that need genuine solid topology
    // (e.g. an inside/outside classifier querying the CAD model directly)
    // have one to query, not just a shell that happens to be watertight.
    return BRepBuilderAPI_MakeSolid(TopoDS::Shell(sewing.SewedShape())).Solid();
}

} // namespace TestSupport
