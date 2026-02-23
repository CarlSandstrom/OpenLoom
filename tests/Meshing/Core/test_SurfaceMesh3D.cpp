#include <gtest/gtest.h>

#include "Meshing/Core/3D/Surface/SurfaceMesh3D.h"

using namespace Meshing;

TEST(SurfaceMesh3DTest, DefaultConstructIsEmpty)
{
    SurfaceMesh3D mesh;
    EXPECT_TRUE(mesh.nodes.empty());
    EXPECT_TRUE(mesh.triangles.empty());
    EXPECT_TRUE(mesh.faceTriangleIds.empty());
    EXPECT_TRUE(mesh.edgeNodeIds.empty());
}

TEST(SurfaceMesh3DTest, StoresNodesAndTriangles)
{
    SurfaceMesh3D mesh;
    mesh.nodes.push_back(Point3D(0.0, 0.0, 0.0));
    mesh.nodes.push_back(Point3D(1.0, 0.0, 0.0));
    mesh.nodes.push_back(Point3D(0.0, 1.0, 0.0));

    mesh.triangles.push_back({0, 1, 2});

    ASSERT_EQ(mesh.nodes.size(), 3u);
    EXPECT_DOUBLE_EQ(mesh.nodes[1].x(), 1.0);

    ASSERT_EQ(mesh.triangles.size(), 1u);
    EXPECT_EQ(mesh.triangles[0][2], 2u);
}

TEST(SurfaceMesh3DTest, FaceGroupsMapCorrectly)
{
    SurfaceMesh3D mesh;
    mesh.faceTriangleIds["Face1"] = {0, 1, 2};
    mesh.faceTriangleIds["Face2"] = {3, 4};

    ASSERT_EQ(mesh.faceTriangleIds.count("Face1"), 1u);
    EXPECT_EQ(mesh.faceTriangleIds.at("Face1").size(), 3u);
    EXPECT_EQ(mesh.faceTriangleIds.at("Face2")[1], 4u);
}

TEST(SurfaceMesh3DTest, EdgeNodesPreserveOrder)
{
    SurfaceMesh3D mesh;
    mesh.edgeNodeIds["Edge1"] = {0, 5, 3, 7, 2};

    ASSERT_EQ(mesh.edgeNodeIds.count("Edge1"), 1u);
    const auto& nodes = mesh.edgeNodeIds.at("Edge1");
    ASSERT_EQ(nodes.size(), 5u);
    EXPECT_EQ(nodes[0], 0u);
    EXPECT_EQ(nodes[2], 3u);
    EXPECT_EQ(nodes[4], 2u);
}
