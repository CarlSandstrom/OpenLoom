#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Meshing/Core/3D/RCDT/RCDTMesher.h"
#include "Meshing/Data/3D/SurfaceMesh3D.h"
#include "Readers/OpenCascade/TopoDS_ShapeConverter.h"

#include <BRepPrimAPI_MakeCylinder.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <numbers>
#include <set>
#include <unordered_set>

#include <gtest/gtest.h>

using namespace Meshing;

// ============================================================================
// RCDTMesherCylinderTest
//
// End-to-end test: mesh a closed OCC cylinder (radius 3, height 8) using
// RCDTMesher directly.  The mesh is built once per suite in SetUpTestSuite
// and shared across all test cases to keep the suite fast.
//
// Covers OPE-136 pass criteria:
//   1. mesh() completes without throwing and produces a non-empty result
//   2. Every boundary edge node participates in at least one triangle
//   3. No duplicate triangles
//   4. No degenerate (zero-area) triangles
//   5. All triangle nodes lie on their respective CAD surface (gap ≤ tolerance)
// ============================================================================

class RCDTMesherCylinderTest : public ::testing::Test
{
protected:
    static constexpr double RADIUS = 3.0;
    static constexpr double HEIGHT = 8.0;
    static constexpr double NODE_ON_SURFACE_TOLERANCE = 1e-3;

    static void SetUpTestSuite()
    {
        gp_Pnt origin(0.0, 0.0, 0.0);
        gp_Dir axisZ(0.0, 0.0, 1.0);
        shape_ = BRepPrimAPI_MakeCylinder(gp_Ax2(origin, axisZ), RADIUS, HEIGHT).Shape();

        converter_ = std::make_unique<Readers::TopoDS_ShapeConverter>(shape_);

        // Edge-only discretization: RCDT handles the surface interior itself.
        const Geometry3D::DiscretizationSettings3D discSettings(
            std::nullopt, std::numbers::pi / 8.0, 0);

        RCDTMesher mesher(converter_->getGeometryCollection(),
                          converter_->getTopology(),
                          discSettings);

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

TopoDS_Shape RCDTMesherCylinderTest::shape_;
std::unique_ptr<Readers::TopoDS_ShapeConverter> RCDTMesherCylinderTest::converter_;
SurfaceMesh3D RCDTMesherCylinderTest::mesh_;

// ============================================================================
// 1. Completes: mesh() must not throw and must produce output
// ============================================================================

TEST_F(RCDTMesherCylinderTest, Completes)
{
    EXPECT_FALSE(mesh_.nodes.empty());
    EXPECT_FALSE(mesh_.triangles.empty());
    EXPECT_FALSE(mesh_.faceTriangleIds.empty());
}

// ============================================================================
// 2. Every boundary edge node participates in at least one triangle
// ============================================================================

TEST_F(RCDTMesherCylinderTest, AllEdgeNodesCovered)
{
    // Build the set of all node IDs that appear in at least one triangle.
    std::unordered_set<size_t> nodesInTriangles;
    for (const auto& triangle : mesh_.triangles)
        for (const size_t nodeId : triangle)
            nodesInTriangles.insert(nodeId);

    for (const auto& [edgeId, nodeIds] : mesh_.edgeNodeIds)
    {
        for (const size_t nodeId : nodeIds)
        {
            EXPECT_TRUE(nodesInTriangles.count(nodeId))
                << "Edge node " << nodeId << " on edge " << edgeId
                << " does not appear in any triangle";
        }
    }
}

// ============================================================================
// 3. No duplicate triangles
// ============================================================================

TEST_F(RCDTMesherCylinderTest, NoDuplicateTriangles)
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
// 4. No degenerate (zero-area) triangles
// ============================================================================

TEST_F(RCDTMesherCylinderTest, NoDegenerateTriangles)
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

// ============================================================================
// 5. All triangle nodes lie on their respective CAD surface
// ============================================================================

TEST_F(RCDTMesherCylinderTest, AllNodesOnCylinderSurface)
{
    const auto& geometry = converter_->getGeometryCollection();

    for (const auto& [faceId, triangleIndices] : mesh_.faceTriangleIds)
    {
        const Geometry3D::ISurface3D* surface = geometry.getSurface(faceId);
        ASSERT_NE(surface, nullptr) << "Surface not found: " << faceId;

        for (const size_t triangleIndex : triangleIndices)
        {
            for (const size_t nodeId : mesh_.triangles[triangleIndex])
            {
                const double gap = surface->getGap(mesh_.nodes[nodeId]);
                EXPECT_LE(gap, NODE_ON_SURFACE_TOLERANCE)
                    << "Node " << nodeId << " has gap " << gap
                    << " > " << NODE_ON_SURFACE_TOLERANCE
                    << " from surface " << faceId;
            }
        }
    }
}
