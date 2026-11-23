#pragma once

#include <TopoDS_Shape.hxx>

/// Solids the 3D examples mesh, shared where more than one example starts from
/// the same part.
namespace ExampleShapes3D
{

// M8 hex nut (metric standard, all dimensions in mm).
constexpr double HEX_NUT_ACROSS_FLATS = 13.0; // flat-to-flat width
constexpr double HEX_NUT_HEIGHT = 6.5;
constexpr double HEX_NUT_BORE_RADIUS = 4.0; // M8 bore, 8 mm diameter

/// The M8 hex nut: a hexagonal prism from z = 0 to HEX_NUT_HEIGHT, one flat
/// facing +X, minus a full-height cylindrical bore on the Z axis.
TopoDS_Shape buildHexNut();

} // namespace ExampleShapes3D
