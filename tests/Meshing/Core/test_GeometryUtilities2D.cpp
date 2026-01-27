#include <gtest/gtest.h>

#include "Common/Types.h"
#include "Meshing/Core/2D/GeometryStructures2D.h"
#include "Meshing/Core/2D/GeometryUtilities2D.h"

#include <cmath>

using namespace Meshing;

namespace
{
constexpr double TOLERANCE = 1e-9;
}

TEST(GeometryUtilities2DTest, ComputeEdgeLengthReturnsCorrectDistance)
{
    Point2D p1(0.0, 0.0);
    Point2D p2(3.0, 4.0);

    double length = GeometryUtilities2D::computeEdgeLength(p1, p2);

    EXPECT_NEAR(length, 5.0, TOLERANCE);
}

TEST(GeometryUtilities2DTest, ComputeEdgeLengthHandlesZeroLength)
{
    Point2D p1(2.0, 3.0);
    Point2D p2(2.0, 3.0);

    double length = GeometryUtilities2D::computeEdgeLength(p1, p2);

    EXPECT_NEAR(length, 0.0, TOLERANCE);
}

TEST(GeometryUtilities2DTest, CreateDiametralCircleFromSegment)
{
    Point2D p1(0.0, 0.0);
    Point2D p2(4.0, 0.0);

    Circle2D circle = GeometryUtilities2D::createDiametralCircle(p1, p2);

    EXPECT_NEAR(circle.center.x(), 2.0, TOLERANCE);
    EXPECT_NEAR(circle.center.y(), 0.0, TOLERANCE);
    EXPECT_NEAR(circle.radius, 2.0, TOLERANCE);
}

TEST(GeometryUtilities2DTest, IsPointInsideCircleDetectsEncroachment)
{
    Point2D p1(0.0, 0.0);
    Point2D p2(4.0, 0.0);
    Circle2D circle = GeometryUtilities2D::createDiametralCircle(p1, p2);

    Point2D inside(2.0, 0.5);
    Point2D outside(2.0, 3.0);
    Point2D onCircle(2.0, 2.0);

    EXPECT_TRUE(GeometryUtilities2D::isPointInsideCircle(circle, inside));
    EXPECT_FALSE(GeometryUtilities2D::isPointInsideCircle(circle, outside));
    EXPECT_FALSE(GeometryUtilities2D::isPointInsideCircle(circle, onCircle));
}

TEST(GeometryUtilities2DTest, IsPointInsideCircleEndpointsAreOnBoundary)
{
    Point2D p1(0.0, 0.0);
    Point2D p2(4.0, 0.0);
    Circle2D circle = GeometryUtilities2D::createDiametralCircle(p1, p2);

    EXPECT_FALSE(GeometryUtilities2D::isPointInsideCircle(circle, p1));
    EXPECT_FALSE(GeometryUtilities2D::isPointInsideCircle(circle, p2));
}

TEST(GeometryUtilities2DTest, IsPointInsideCircleDetectsContainment)
{
    Circle2D circle;
    circle.center = Point2D(0.0, 0.0);
    circle.radius = 5.0;

    Point2D inside(3.0, 0.0);
    Point2D outside(6.0, 0.0);
    Point2D onCircle(5.0, 0.0);

    EXPECT_TRUE(GeometryUtilities2D::isPointInsideCircle(circle, inside));
    EXPECT_FALSE(GeometryUtilities2D::isPointInsideCircle(circle, outside));
    // On the circle should be considered outside due to tolerance
    EXPECT_FALSE(GeometryUtilities2D::isPointInsideCircle(circle, onCircle));
}

TEST(GeometryUtilities2DTest, CreateSuperTriangleContainsAllPoints)
{
    std::vector<Point2D> points = {
        Point2D(0.0, 0.0),
        Point2D(1.0, 0.0),
        Point2D(0.5, 0.5),
        Point2D(0.2, 0.8)};

    auto superTri = GeometryUtilities2D::createSuperTriangle(points);

    // Verify that all original points are inside the super triangle
    // This is a simplified check - a full test would verify point-in-triangle
    EXPECT_EQ(superTri.size(), 3u);

    // The super triangle should have a large enough extent
    double minX = std::min({superTri[0].x(), superTri[1].x(), superTri[2].x()});
    double maxX = std::max({superTri[0].x(), superTri[1].x(), superTri[2].x()});
    double minY = std::min({superTri[0].y(), superTri[1].y(), superTri[2].y()});
    double maxY = std::max({superTri[0].y(), superTri[1].y(), superTri[2].y()});

    // All original points should be within the super triangle bounds
    for (const auto& pt : points)
    {
        EXPECT_LT(pt.x(), maxX);
        EXPECT_GT(pt.x(), minX);
        EXPECT_LT(pt.y(), maxY);
        EXPECT_GT(pt.y(), minY);
    }
}
