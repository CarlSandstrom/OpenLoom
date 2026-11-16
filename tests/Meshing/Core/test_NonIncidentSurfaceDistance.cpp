#include "Meshing/Core/3D/General/NonIncidentSurfaceDistance.h"

#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/IEdge3D.h"
#include "Readers/OpenCascade/TopoDS_ShapeConverter.h"
#include "Topology/SeamCollection.h"
#include "Topology/Topology3D.h"

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>

#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

using namespace Meshing;

namespace
{

constexpr double RELATIVE_TOLERANCE = 0.01;

/// The corners of the edge running between the two points, in either direction.
std::vector<std::string> cornersOfEdgeBetween(const Readers::TopoDS_ShapeConverter& converter,
                                              const Point3D& end0,
                                              const Point3D& end1)
{
    auto& topology = converter.getTopology();
    for (auto& edgeId : topology.getAllEdgeIds())
    {
        if (topology.getSeamCollection().isSeamTwin(edgeId)) continue;
        auto edge = converter.getGeometryCollection().getEdge(edgeId);
        auto start = edge->getStartPoint();
        auto end = edge->getEndPoint();
        auto matches = ((start - end0).norm() < 1e-9 && (end - end1).norm() < 1e-9) ||
                       ((start - end1).norm() < 1e-9 && (end - end0).norm() < 1e-9);
        if (!matches) continue;
        auto& topologyEdge = topology.getEdge(edgeId);
        return {topologyEdge.getStartCornerId(), topologyEdge.getEndCornerId()};
    }
    ADD_FAILURE() << "no edge between " << end0.transpose() << " and " << end1.transpose();
    return {};
}

/// A 10x10x10 box with a 6x6x9 pocket cut from below: the pocket ceiling at
/// z = 9 lies 1 mm under the top face.
TopoDS_Shape thinCapBox()
{
    auto box = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();
    auto pocket = BRepPrimAPI_MakeBox(gp_Pnt(2.0, 2.0, 0.0), 6.0, 6.0, 9.0).Shape();
    return BRepAlgoAPI_Cut(box, pocket).Shape();
}

} // namespace

TEST(NonIncidentSurfaceDistanceTest, CeilingEdgeIsTheCapThicknessBelowTheTopFace)
{
    Readers::TopoDS_ShapeConverter converter(thinCapBox());
    NonIncidentSurfaceDistance surfaceDistance(converter.getGeometryCollection(), converter.getTopology());

    auto corners = cornersOfEdgeBetween(converter, Point3D(2.0, 2.0, 9.0), Point3D(8.0, 2.0, 9.0));

    EXPECT_NEAR(surfaceDistance.distance(Point3D(5.0, 2.0, 9.0), corners), 1.0, RELATIVE_TOLERANCE);
}

// The nearest point of the pocket ceiling is on its rim, which the surface
// grid does not resolve: the edge search has to find it.
TEST(NonIncidentSurfaceDistanceTest, TopEdgeIsNearestToTheRimOfTheCeiling)
{
    Readers::TopoDS_ShapeConverter converter(thinCapBox());
    NonIncidentSurfaceDistance surfaceDistance(converter.getGeometryCollection(), converter.getTopology());

    auto corners = cornersOfEdgeBetween(converter, Point3D(0.0, 0.0, 10.0), Point3D(10.0, 0.0, 10.0));

    auto expected = std::sqrt(5.0);
    EXPECT_NEAR(surfaceDistance.distance(Point3D(5.0, 0.0, 10.0), corners), expected, RELATIVE_TOLERANCE * expected);
}

// Four of the box's faces touch the edge or one of its ends and are at
// distance zero or five; only the two opposite faces count.
TEST(NonIncidentSurfaceDistanceTest, FacesSharingACornerWithTheEdgeDoNotCount)
{
    Readers::TopoDS_ShapeConverter converter(BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape());
    NonIncidentSurfaceDistance surfaceDistance(converter.getGeometryCollection(), converter.getTopology());

    auto corners = cornersOfEdgeBetween(converter, Point3D(0.0, 0.0, 0.0), Point3D(10.0, 0.0, 0.0));

    EXPECT_NEAR(surfaceDistance.distance(Point3D(5.0, 0.0, 0.0), corners), 10.0, RELATIVE_TOLERANCE * 10.0);
}

// The top circle is a closed edge on the seam's corner; the side and the top
// disc contain that corner, leaving the bottom disc a full height away.
TEST(NonIncidentSurfaceDistanceTest, CylinderTopCircleIsTheHeightAboveTheBottomDisc)
{
    Readers::TopoDS_ShapeConverter converter(BRepPrimAPI_MakeCylinder(5.0, 10.0).Shape());
    NonIncidentSurfaceDistance surfaceDistance(converter.getGeometryCollection(), converter.getTopology());

    auto corners = cornersOfEdgeBetween(converter, Point3D(5.0, 0.0, 10.0), Point3D(5.0, 0.0, 10.0));

    EXPECT_NEAR(surfaceDistance.distance(Point3D(-5.0, 0.0, 10.0), corners), 10.0, RELATIVE_TOLERANCE * 10.0);
}
