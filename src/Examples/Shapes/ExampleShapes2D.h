#pragma once

#include "Common/Types.h"
#include "Topology2D/Corner2D.h"
#include "Topology2D/Edge2D.h"

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

namespace Geometry2D
{
class GeometryCollection2D;
} // namespace Geometry2D

/// Boundary pieces the 2D examples build their domains from. Each adds its
/// corners and edges to the geometry and topology being assembled, named
/// <prefix>_c<i> and <prefix>_e<i>, and returns its edge ids in loop order.
namespace ExampleShapes2D
{

using TopologyCorners = std::unordered_map<std::string, Topology2D::Corner2D>;
using TopologyEdges = std::unordered_map<std::string, Topology2D::Edge2D>;

/// The axis-aligned square [origin, origin + size]^2, edges counter-clockwise
/// from the bottom: <prefix>_e0 bottom, _e1 right, _e2 top, _e3 left.
std::vector<std::string> addSquare(Geometry2D::GeometryCollection2D& geometry,
                                   TopologyCorners& topologyCorners,
                                   TopologyEdges& topologyEdges,
                                   const std::string& prefix,
                                   double size,
                                   const Meshing::Point2D& origin = Meshing::Point2D(0.0, 0.0));

/// A circle of segmentCount arcs, counter-clockwise from angle 0. Pass the
/// returned loop as a hole to cut the disc out, or leave it out of the loops to
/// keep the circle as an internal constraint.
std::vector<std::string> addCircle(Geometry2D::GeometryCollection2D& geometry,
                                   TopologyCorners& topologyCorners,
                                   TopologyEdges& topologyEdges,
                                   const std::string& prefix,
                                   const Meshing::Point2D& center,
                                   double radius,
                                   std::size_t segmentCount);

} // namespace ExampleShapes2D
