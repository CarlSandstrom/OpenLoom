#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/RCDT/RCDTMesher.h"
#include "Meshing/Data/3D/SurfaceMesh3D.h"
#include "Readers/OpenCascade/TopoDS_ShapeConverter.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <TopoDS_Shape.hxx>

#include <algorithm>
#include <array>
#include <set>
#include <unordered_set>

#include <gtest/gtest.h>

using namespace Meshing;

// ============================================================================
// RCDTMesherBoxTest
//
// End-to-end test: mesh a unit box using RCDTMesher.
// A box has 6 flat faces, 12 straight edges, 8 corners — the simplest
// non-trivial OCC solid for validating the full RCDT pipeline.
//
// Checks:
//   1. mesh() completes without throwing and produces non-empty output
//   2. The output covers all 6 faces
//   3. Every boundary edge node participates in at least one triangle
//   4. No duplicate triangles
//   5. No degenerate (zero-area) triangles
// ============================================================================

class RCDTMesherBoxTest : public ::testing::Test
{
protected:
    static constexpr double SIZE = 1.0;
    static constexpr int SEGMENTS_PER_EDGE = 2;

    static void SetUpTestSuite()
    {
        shape_ = BRepPrimAPI_MakeBox(SIZE, SIZE, SIZE).Shape();

        converter_ = std::make_unique<Readers::TopoDS_ShapeConverter>(shape_);

        const Geometry3D::DiscretizationSettings3D discretizationSettings(
            SEGMENTS_PER_EDGE, 0);

        RCDTMesher mesher(converter_->getGeometryCollection(),
                          converter_->getTopology(),
                          discretizationSettings);

        mesh_ = mesher.mesh();
    }

    static void TearDownTestSuite()
    {
        converter_.reset();
    }

    static TopoDS_Shape shape_;
    static std::unique_ptr<Readers::TopoDS_ShapeConverter> converter_;
    static SurfaceMesh3D mesh_;
};

TopoDS_Shape RCDTMesherBoxTest::shape_;
std::unique_ptr<Readers::TopoDS_ShapeConverter> RCDTMesherBoxTest::converter_;
SurfaceMesh3D RCDTMesherBoxTest::mesh_;

// ============================================================================
// 1. Completes: mesh() must not throw and must produce output
// ============================================================================

TEST_F(RCDTMesherBoxTest, Completes)
{
    EXPECT_FALSE(mesh_.nodes.empty());
    EXPECT_FALSE(mesh_.triangles.empty());
    EXPECT_FALSE(mesh_.faceTriangleIds.empty());
}

// ============================================================================
// 2. All 6 faces are covered
// ============================================================================

TEST_F(RCDTMesherBoxTest, AllSixFacesCovered)
{
    EXPECT_EQ(mesh_.faceTriangleIds.size(), 6u);
    for (const auto& [faceId, triangleIndices] : mesh_.faceTriangleIds)
        EXPECT_FALSE(triangleIndices.empty()) << "Face " << faceId << " has no triangles";
}

// ============================================================================
// 3. Every boundary edge node participates in at least one triangle
// ============================================================================

TEST_F(RCDTMesherBoxTest, AllEdgeNodesCovered)
{
    std::unordered_set<size_t> nodesInTriangles;
    for (const auto& triangle : mesh_.triangles)
        for (const size_t nodeId : triangle)
            nodesInTriangles.insert(nodeId);

    for (const auto& [edgeId, nodeIds] : mesh_.edgeNodeIds)
        for (const size_t nodeId : nodeIds)
            EXPECT_TRUE(nodesInTriangles.count(nodeId))
                << "Edge node " << nodeId << " on edge " << edgeId
                << " does not appear in any triangle";
}

// ============================================================================
// 4. No duplicate triangles
// ============================================================================

TEST_F(RCDTMesherBoxTest, NoDuplicateTriangles)
{
    std::set<std::array<size_t, 3>> seen;
    for (auto triangle : mesh_.triangles)
    {
        std::sort(triangle.begin(), triangle.end());
        EXPECT_TRUE(seen.insert(triangle).second)
            << "Duplicate triangle: {" << triangle[0] << ", " << triangle[1]
            << ", " << triangle[2] << "}";
    }
}

// ============================================================================
// 5. Correct triangle count — 6 triangles per face (8 boundary-only nodes → n−2)
// ============================================================================

TEST_F(RCDTMesherBoxTest, TriangleCountPerFace)
{
    for (const auto& [faceId, triangleIndices] : mesh_.faceTriangleIds)
        EXPECT_EQ(triangleIndices.size(), 6u)
            << "Face " << faceId << " has " << triangleIndices.size()
            << " triangles (expected 6)";

    const size_t totalTriangles = mesh_.triangles.size();
    EXPECT_EQ(totalTriangles, 36u)
        << "Expected 36 total triangles (6 faces × 6), got " << totalTriangles;
}

// ============================================================================
// 6. No degenerate (zero-area) triangles
// ============================================================================

TEST_F(RCDTMesherBoxTest, NoDegenerateTriangles)
{
    for (const auto& triangle : mesh_.triangles)
    {
        const Point3D& a = mesh_.nodes[triangle[0]];
        const Point3D& b = mesh_.nodes[triangle[1]];
        const Point3D& c = mesh_.nodes[triangle[2]];
        const double area = 0.5 * (b - a).cross(c - a).norm();
        EXPECT_GT(area, 1e-10)
            << "Degenerate triangle: {" << triangle[0] << ", " << triangle[1]
            << ", " << triangle[2] << "} area=" << area;
    }
}
