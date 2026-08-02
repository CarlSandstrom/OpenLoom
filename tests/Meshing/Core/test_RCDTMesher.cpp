#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Meshing/Core/3D/RCDT/RCDTMesher.h"
#include "Meshing/Data/3D/SurfaceMesh3D.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"
#include "Readers/OpenCascade/TopoDS_ShapeConverter.h"

#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
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
        // Angle threshold pi*2/20 ≈ 18.6° yields ~20 points per circular edge,
        // giving a moderately fine boundary polygon for each cap.
        const Geometry3D::DiscretizationSettings3D discSettings(
            std::nullopt, std::numbers::pi * 2.0 / 20.0 + 0.01, 0);

        // Quality settings: circumradius/shortest-edge ratio ≤ 1.0 (the
        // default) and chord deviation ≤ 0.5.  The chord deviation for a
        // ~18.6° arc on a radius-3 cylinder is ~0.04 m, well under the 0.5
        // bound, so only the ratio criterion drives refinement here.
        SurfaceMesh3DQualitySettings qualitySettings;
        qualitySettings.chordDeviationTolerance = 0.5;

        RCDTMesher mesher(converter_->getGeometryCollection(),
                          converter_->getTopology(),
                          discSettings,
                          qualitySettings);

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

// ============================================================================
// RCDTMesherSphereTest
//
// End-to-end test: mesh a closed OCC sphere (radius 3) using RCDTMesher
// directly. Covers OPE-165.
//
// A sphere is topologically the minimal closed single-face solid: one face,
// one real (meridian) edge plus its seam twin, and two degenerate polar
// edges collapsed to a point at each pole. This is a harder seed case than
// the cylinder: with edge-only discretization the only 3D-spanning boundary
// data is a single meridian arc, which is not enough for RCDTMesher::mesh()
// to find any restricted faces at all (buildInitial reports 0), so this test
// requires surface interior samples (DiscretizationSettings3D's third
// argument) to seed the ambient Delaunay with genuine 3D coverage.
//
// Also exercises degenerate-edge handling (BoundaryDiscretizer3D and
// CurveSegmentManager both skip IEdge3D::isDegenerate() edges) — before that
// fix, angle-based discretization walked the polar edges as if they were
// real curves and inserted ~1000 spurious coincident points per pole.
//
// Covers the same pass criteria as the cylinder test (OPE-136):
//   1. mesh() completes without throwing and produces a non-empty result
//   2. Every boundary edge node participates in at least one triangle
//   3. No duplicate triangles
//   4. No degenerate (zero-area) triangles
//   5. All triangle nodes lie on the sphere surface (gap ≤ tolerance)
// ============================================================================

class RCDTMesherSphereTest : public ::testing::Test
{
protected:
    static constexpr double RADIUS = 3.0;
    static constexpr double NODE_ON_SURFACE_TOLERANCE = 1e-3;

    static void SetUpTestSuite()
    {
        shape_ = BRepPrimAPI_MakeSphere(RADIUS).Shape();

        converter_ = std::make_unique<Readers::TopoDS_ShapeConverter>(shape_);

        // Angle-based edge discretization (same resolution as the cylinder
        // test) plus 5 surface interior samples/direction — needed because
        // the sphere's boundary alone (one meridian arc + 2 poles) does not
        // span the surface in 3D. See class comment above.
        const Geometry3D::DiscretizationSettings3D discSettings(
            std::nullopt, std::numbers::pi * 2.0 / 20.0 + 0.01, 5);

        SurfaceMesh3DQualitySettings qualitySettings;
        qualitySettings.chordDeviationTolerance = 0.5;

        RCDTMesher mesher(converter_->getGeometryCollection(),
                          converter_->getTopology(),
                          discSettings,
                          qualitySettings);

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

TopoDS_Shape RCDTMesherSphereTest::shape_;
std::unique_ptr<Readers::TopoDS_ShapeConverter> RCDTMesherSphereTest::converter_;
SurfaceMesh3D RCDTMesherSphereTest::mesh_;

TEST_F(RCDTMesherSphereTest, Completes)
{
    EXPECT_FALSE(mesh_.nodes.empty());
    EXPECT_FALSE(mesh_.triangles.empty());
    EXPECT_FALSE(mesh_.faceTriangleIds.empty());
}

TEST_F(RCDTMesherSphereTest, AllEdgeNodesCovered)
{
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

TEST_F(RCDTMesherSphereTest, NoDuplicateTriangles)
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

TEST_F(RCDTMesherSphereTest, NoDegenerateTriangles)
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

TEST_F(RCDTMesherSphereTest, AllNodesOnSphereSurface)
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
