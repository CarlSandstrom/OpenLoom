#include <gtest/gtest.h>

#include "../Meshing/Core/SaddleShape.h"

#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/IEdge3D.h"
#include "Readers/OpenCascade/TopoDS_ShapeConverter.h"
#include "Topology/Topology3D.h"

#include <TopoDS_Shape.hxx>

namespace
{

constexpr std::size_t POLYLINE_SEGMENTS = 20000;
constexpr double RELATIVE_TOLERANCE = 1e-6;

/// Arc length from parameterStart to parameter, by a fine polyline -- an
/// estimate independent of OpenCascade's arc-length integration.
double polylineLength(const Geometry3D::IEdge3D& edge, double parameterStart, double parameter)
{
    double length = 0.0;
    auto previous = edge.getPoint(parameterStart);
    for (std::size_t i = 1; i <= POLYLINE_SEGMENTS; ++i)
    {
        auto point = edge.getPoint(parameterStart + (parameter - parameterStart) * static_cast<double>(i) /
                                                        static_cast<double>(POLYLINE_SEGMENTS));
        length += (point - previous).norm();
        previous = point;
    }
    return length;
}

} // namespace

// The saddle's crease arcs are B-spline edges. OpenCascade's arc-length
// integration without an explicit tolerance measured the top arcs 1% long
// (9.394 against 9.294), and the inverse lookup missed by up to 0.02, enough
// for ProtectingBallPlacer to accept neighbouring balls that leave the crease
// between them uncovered (OPE-219).
TEST(OpenCascadeEdgeTest, ArcLengthMatchesThePolylineOnBSplineCreases)
{
    Readers::TopoDS_ShapeConverter converter(TestSupport::buildSaddleSolid());
    auto& geometry = converter.getGeometryCollection();

    for (auto& edgeId : converter.getTopology().getAllEdgeIds())
    {
        auto edge = geometry.getEdge(edgeId);
        if (!edge || edge->isDegenerate()) continue;

        auto [start, end] = edge->getParameterBounds();
        auto length = edge->getLength();
        EXPECT_NEAR(length, polylineLength(*edge, start, end), RELATIVE_TOLERANCE * length) << edgeId;

        for (double fraction : {0.1, 0.33, 0.5, 0.66, 0.9, 1.0})
        {
            auto parameter = edge->getParameterAtArcLengthFraction(start, end, fraction);
            EXPECT_NEAR(polylineLength(*edge, start, parameter), fraction * length, RELATIVE_TOLERANCE * length)
                << edgeId << " at fraction " << fraction;
        }
    }
}
