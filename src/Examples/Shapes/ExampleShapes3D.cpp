#include "ExampleShapes3D.h"

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <cmath>
#include <numbers>

namespace ExampleShapes3D
{

TopoDS_Shape buildHexNut()
{
    // The circumscribed radius (vertex-to-center) for a regular hexagon with
    // flat-to-flat distance ACROSS_FLATS: r = (ACROSS_FLATS / 2) / cos(30°) = ACROSS_FLATS / sqrt(3).
    double circumscribedRadius = HEX_NUT_ACROSS_FLATS / std::sqrt(3.0);

    // Hexagonal base: vertices at 30°, 90°, 150°, 210°, 270°, 330° place one flat
    // face toward +X (i.e., the right-hand flat is vertical, parallel to Y).
    BRepBuilderAPI_MakePolygon hexPolygon;
    for (int cornerIndex = 0; cornerIndex < 6; ++cornerIndex)
    {
        double angle = (2.0 * cornerIndex + 1.0) * std::numbers::pi / 6.0;
        hexPolygon.Add(gp_Pnt(circumscribedRadius * std::cos(angle), circumscribedRadius * std::sin(angle), 0.0));
    }
    hexPolygon.Close();

    TopoDS_Face hexFace = BRepBuilderAPI_MakeFace(hexPolygon.Wire()).Face();
    TopoDS_Shape hexPrism = BRepPrimAPI_MakePrism(hexFace, gp_Vec(0.0, 0.0, HEX_NUT_HEIGHT)).Shape();

    gp_Ax2 boreAxis(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0));
    TopoDS_Shape bore = BRepPrimAPI_MakeCylinder(boreAxis, HEX_NUT_BORE_RADIUS, HEX_NUT_HEIGHT).Shape();
    return BRepAlgoAPI_Cut(hexPrism, bore).Shape();
}

} // namespace ExampleShapes3D
