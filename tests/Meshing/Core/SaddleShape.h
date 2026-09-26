#pragma once

class TopoDS_Shape;

namespace TestSupport
{

/// Half-extent of the saddle domain in x and y. The crease arcs at
/// x = +/- SADDLE_HALF_EXTENT are where the horn tips sit.
inline constexpr double SADDLE_HALF_EXTENT = 2.0;

/// The saddle solid used by `src/Examples/3D/Surface/SaddleSurfaceMesh.cc`:
/// the hyperbolic paraboloid z = x^2 - y^2 as the top face, a flat bottom, and
/// four ruled side faces, sewn into a genuine TopoDS_Solid.
///
/// Duplicated from the example rather than shared with it, because the example
/// owns its own shape and moving it into the library would put a test fixture
/// in production code. The two must stay in step: a test built on this shape
/// asserts things about specific features of that geometry, so a change to the
/// example that is not mirrored here silently stops testing the example.
TopoDS_Shape buildSaddleSolid();

} // namespace TestSupport
