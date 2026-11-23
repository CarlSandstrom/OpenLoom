#include "ExampleShapes2D.h"

#include "Geometry/2D/Base/Corner2D.h"
#include "Geometry/2D/Base/GeometryCollection2D.h"
#include "Geometry/2D/Base/LinearEdge2D.h"
#include "Geometry/2D/OpenCascade/OpenCascade2DCorner.h"
#include "Geometry/2D/OpenCascade/OpenCascade2DEdge.h"

#include <Geom2d_Circle.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <gp_Ax2d.hxx>
#include <gp_Circ2d.hxx>
#include <gp_Dir2d.hxx>
#include <gp_Pnt2d.hxx>

#include <array>
#include <cmath>
#include <memory>
#include <numbers>

namespace ExampleShapes2D
{

namespace
{
constexpr std::size_t SQUARE_CORNER_COUNT = 4;
constexpr double FULL_TURN = 2.0 * std::numbers::pi;
} // namespace

using Meshing::Point2D;

std::vector<std::string> addSquare(Geometry2D::GeometryCollection2D& geometry,
                                   TopologyCorners& topologyCorners,
                                   TopologyEdges& topologyEdges,
                                   const std::string& prefix,
                                   double size,
                                   const Point2D& origin)
{
    std::array<Point2D, SQUARE_CORNER_COUNT> corners = {origin,
                                                        Point2D(origin.x() + size, origin.y()),
                                                        Point2D(origin.x() + size, origin.y() + size),
                                                        Point2D(origin.x(), origin.y() + size)};

    std::vector<std::string> edgeLoop;
    for (std::size_t index = 0; index < SQUARE_CORNER_COUNT; ++index)
    {
        auto nextIndex = (index + 1) % SQUARE_CORNER_COUNT;
        auto cornerId = prefix + "_c" + std::to_string(index);
        auto edgeId = prefix + "_e" + std::to_string(index);
        auto previousEdgeId = prefix + "_e" + std::to_string((index + SQUARE_CORNER_COUNT - 1) % SQUARE_CORNER_COUNT);

        geometry.addCorner(std::make_unique<Geometry2D::Corner2D>(cornerId, corners[index]));
        geometry.addEdge(std::make_unique<Geometry2D::LinearEdge2D>(edgeId, corners[index], corners[nextIndex]));

        topologyCorners.emplace(cornerId, Topology2D::Corner2D(cornerId, {edgeId, previousEdgeId}));
        topologyEdges.emplace(edgeId, Topology2D::Edge2D(edgeId, cornerId, prefix + "_c" + std::to_string(nextIndex)));
        edgeLoop.push_back(edgeId);
    }
    return edgeLoop;
}

// All corners first, then all arcs: the order every example used, which the
// geometry and topology maps' iteration order -- and so the mesh -- depends on.
std::vector<std::string> addCircle(Geometry2D::GeometryCollection2D& geometry,
                                   TopologyCorners& topologyCorners,
                                   TopologyEdges& topologyEdges,
                                   const std::string& prefix,
                                   const Point2D& center,
                                   double radius,
                                   std::size_t segmentCount)
{
    gp_Ax2d axis(gp_Pnt2d(center.x(), center.y()), gp_Dir2d(1.0, 0.0));
    gp_Circ2d circle(axis, radius);

    for (std::size_t index = 0; index < segmentCount; ++index)
    {
        double angle = FULL_TURN * index / segmentCount;
        double x = center.x() + radius * std::cos(angle);
        double y = center.y() + radius * std::sin(angle);

        auto cornerId = prefix + "_c" + std::to_string(index);
        auto edgeId = prefix + "_e" + std::to_string(index);
        auto previousEdgeId = prefix + "_e" + std::to_string((index + segmentCount - 1) % segmentCount);

        geometry.addCorner(std::make_unique<Geometry2D::OpenCascade2DCorner>(gp_Pnt2d(x, y), cornerId));
        topologyCorners.emplace(cornerId, Topology2D::Corner2D(cornerId, {edgeId, previousEdgeId}));
    }

    std::vector<std::string> edgeLoop;
    for (std::size_t index = 0; index < segmentCount; ++index)
    {
        double startAngle = FULL_TURN * index / segmentCount;
        double endAngle = FULL_TURN * (index + 1) / segmentCount;

        Handle(Geom2d_Circle) circleGeometry = new Geom2d_Circle(circle);
        Handle(Geom2d_TrimmedCurve) arc = new Geom2d_TrimmedCurve(circleGeometry, startAngle, endAngle);

        auto edgeId = prefix + "_e" + std::to_string(index);
        auto startCornerId = prefix + "_c" + std::to_string(index);
        auto endCornerId = prefix + "_c" + std::to_string((index + 1) % segmentCount);

        geometry.addEdge(std::make_unique<Geometry2D::OpenCascade2DEdge>(arc, edgeId));
        topologyEdges.emplace(edgeId, Topology2D::Edge2D(edgeId, startCornerId, endCornerId));
        edgeLoop.push_back(edgeId);
    }
    return edgeLoop;
}

} // namespace ExampleShapes2D
