#include <gtest/gtest.h>

#include "Common/Types.h"
#include "Meshing/Core/2D/ConstrainedDelaunay2D.h"

#include <unordered_map>
#include <vector>

using namespace Meshing;

/*
TEST(ConstrainedDelaunay2D, TriangulatesSimpleSquare)
{
    // Create a simple square with 4 corner points
    std::unordered_map<size_t, Point2D> nodeCoords;
    nodeCoords[0] = Point2D(0.0, 0.0);
    nodeCoords[1] = Point2D(1.0, 0.0);
    nodeCoords[2] = Point2D(1.0, 1.0);
    nodeCoords[3] = Point2D(0.0, 1.0);

    ConstrainedDelaunay2D delaunay(nodeCoords);

    // Add constraint edges for the square boundary
    delaunay.addConstraintEdge(0, 1);
    delaunay.addConstraintEdge(1, 2);
    delaunay.addConstraintEdge(2, 3);
    delaunay.addConstraintEdge(3, 0);

    // Triangulate
    auto triangles = delaunay.triangulate();

    // A square should be split into 2 triangles
    EXPECT_EQ(triangles.size(), 2u);

    // Verify all triangles have 3 nodes
    for (const auto& tri : triangles)
    {
        EXPECT_EQ(tri.size(), 3u);
    }
}

TEST(ConstrainedDelaunay2D, TriangulatesSquareWithInteriorPoint)
{
    // Create a square with a center point
    std::unordered_map<size_t, Point2D> nodeCoords;
    nodeCoords[0] = Point2D(0.0, 0.0);
    nodeCoords[1] = Point2D(1.0, 0.0);
    nodeCoords[2] = Point2D(1.0, 1.0);
    nodeCoords[3] = Point2D(0.0, 1.0);
    nodeCoords[4] = Point2D(0.5, 0.5); // Center point

    ConstrainedDelaunay2D delaunay(nodeCoords);

    // Add constraint edges for the square boundary
    delaunay.addConstraintEdge(0, 1);
    delaunay.addConstraintEdge(1, 2);
    delaunay.addConstraintEdge(2, 3);
    delaunay.addConstraintEdge(3, 0);

    // Triangulate
    auto triangles = delaunay.triangulate();

    // A square with one interior point should create 4 triangles
    EXPECT_EQ(triangles.size(), 4u);

    // Verify the center point is used in multiple triangles
    int centerUsageCount = 0;
    for (const auto& tri : triangles)
    {
        for (size_t nodeId : tri)
        {
            if (nodeId == 4)
            {
                centerUsageCount++;
                break;
            }
        }
    }
    EXPECT_EQ(centerUsageCount, 4);
}

TEST(ConstrainedDelaunay2D, HandlesConcavePolygon)
{
    // Create an L-shaped polygon (concave)
    std::unordered_map<size_t, Point2D> nodeCoords;
    nodeCoords[0] = Point2D(0.0, 0.0);
    nodeCoords[1] = Point2D(1.0, 0.0);
    nodeCoords[2] = Point2D(1.0, 0.5);
    nodeCoords[3] = Point2D(0.5, 0.5);
    nodeCoords[4] = Point2D(0.5, 1.0);
    nodeCoords[5] = Point2D(0.0, 1.0);

    ConstrainedDelaunay2D delaunay(nodeCoords);

    // Add constraint edges for the L-shape boundary
    delaunay.addConstraintEdge(0, 1);
    delaunay.addConstraintEdge(1, 2);
    delaunay.addConstraintEdge(2, 3);
    delaunay.addConstraintEdge(3, 4);
    delaunay.addConstraintEdge(4, 5);
    delaunay.addConstraintEdge(5, 0);

    // Triangulate
    auto triangles = delaunay.triangulate();

    // Should successfully triangulate (exact count depends on algorithm)
    EXPECT_GE(triangles.size(), 4u); // At least 4 triangles for L-shape
    EXPECT_LE(triangles.size(), 6u); // At most 6 triangles

    // Verify all triangles have 3 nodes
    for (const auto& tri : triangles)
    {
        EXPECT_EQ(tri.size(), 3u);
    }
}

TEST(ConstrainedDelaunay2D, HandlesEmptyInput)
{
    // Test with no nodes
    std::unordered_map<size_t, Point2D> nodeCoords;

    ConstrainedDelaunay2D delaunay(nodeCoords);

    auto triangles = delaunay.triangulate();

    EXPECT_EQ(triangles.size(), 0u);
}

TEST(ConstrainedDelaunay2D, HandlesInsufficientPoints)
{
    // Test with only 2 points
    std::unordered_map<size_t, Point2D> nodeCoords;
    nodeCoords[0] = Point2D(0.0, 0.0);
    nodeCoords[1] = Point2D(1.0, 0.0);

    ConstrainedDelaunay2D delaunay(nodeCoords);

    auto triangles = delaunay.triangulate();

    EXPECT_EQ(triangles.size(), 0u);
}

TEST(ConstrainedDelaunay2D, TriangulatesTriangle)
{
    // Test with exactly 3 points (already a triangle)
    std::unordered_map<size_t, Point2D> nodeCoords;
    nodeCoords[0] = Point2D(0.0, 0.0);
    nodeCoords[1] = Point2D(1.0, 0.0);
    nodeCoords[2] = Point2D(0.5, 1.0);

    ConstrainedDelaunay2D delaunay(nodeCoords);

    // Add constraint edges
    delaunay.addConstraintEdge(0, 1);
    delaunay.addConstraintEdge(1, 2);
    delaunay.addConstraintEdge(2, 0);

    auto triangles = delaunay.triangulate();

    // Should create exactly 1 triangle
    EXPECT_EQ(triangles.size(), 1u);

    // Verify it uses all 3 nodes
    const auto& tri = triangles[0];
    EXPECT_EQ(tri.size(), 3u);
}
*/