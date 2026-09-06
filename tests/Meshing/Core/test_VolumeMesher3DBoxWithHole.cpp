#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/General/SizingFieldBuilder3D.h"
#include "Meshing/Core/3D/Volume/VolumeMesher3D.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"
#include "Meshing/Data/3D/VolumeMesh3D.h"
#include "Readers/OpenCascade/TopoDS_ShapeConverter.h"

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <memory>

#include <gtest/gtest.h>

using namespace Meshing;

// ============================================================================
// VolumeMesher3DBoxWithHoleTest
//
// End-to-end regression test for the RCDT volume pipeline on a solid with an
// internal boundary: a box with a cylindrical hole drilled through it — the
// same model as the BoxWithHole example, which is what OPE-169/170 were
// verified against.
//
// This exists because that verified result silently regressed to an empty
// mesh and stayed regressed for weeks with the whole suite green (OPE-185):
// nothing anywhere asserted a non-empty volume for a solid with a hole. The
// failure mode is all-or-nothing rather than gradual — AmbientTetrahedronClassifier
// floods the ambient region inward, so a single unclosed boundary edge connects
// the ambient region to the solid one and every tetrahedron is classified
// ambient. Partial progress does not produce a partially-correct mesh, it
// produces no mesh, which is why "non-empty" is a meaningful assertion here
// and not a triviality.
//
// Checks:
//   1. The mesh is non-empty (the regression's headline symptom)
//   2. Its boundary is watertight — every boundary edge shared by exactly 2
//      triangles
//   3. Its volume matches the analytic value for box-minus-cylinder
//   4. Every declared boundary face group carries triangles
// ============================================================================

class VolumeMesher3DBoxWithHoleTest : public ::testing::Test
{
protected:
    static constexpr double BOX_SIZE = 10.0;
    static constexpr double BORE_RADIUS = 2.0;
    static constexpr int SEGMENTS_PER_EDGE = 3;
    static constexpr int SEGMENTS_PER_CURVED_EDGE = 2;

    /// Volume of the box minus the volume of the cylinder drilled through it.
    static double analyticVolume()
    {
        return BOX_SIZE * BOX_SIZE * BOX_SIZE - M_PI * BORE_RADIUS * BORE_RADIUS * BOX_SIZE;
    }

    static void SetUpTestSuite()
    {
        const TopoDS_Shape box = BRepPrimAPI_MakeBox(BOX_SIZE, BOX_SIZE, BOX_SIZE).Shape();

        const gp_Ax2 axis(gp_Pnt(0.5 * BOX_SIZE, 0.5 * BOX_SIZE, 0.0), gp_Dir(0.0, 0.0, 1.0));
        const TopoDS_Shape cylinder = BRepPrimAPI_MakeCylinder(axis, BORE_RADIUS, BOX_SIZE).Shape();

        shape_ = BRepAlgoAPI_Cut(box, cylinder).Shape();
        converter_ = std::make_unique<Readers::TopoDS_ShapeConverter>(shape_);

        const Geometry3D::DiscretizationSettings3D discretizationSettings(
            SEGMENTS_PER_EDGE, SEGMENTS_PER_CURVED_EDGE);

        // The sizing field is load-bearing here, not incidental (OPE-185):
        // without it the bore's crease and seam discretize into 2.0-long
        // segments against a 0.33 refinement target, their protecting balls
        // span several elements, and the frozen band around the bore leaves
        // holes that make this mesh empty. It is opt-in rather than the
        // default because enabling it globally regresses the surface path
        // via OPE-184 (SaddleSurfaceMesh 17 -> 222 defects).
        VolumeMesher3D mesher(converter_->getGeometryCollection(),
                              converter_->getTopology(),
                              discretizationSettings,
                              SurfaceMesh3DQualitySettings{},
                              SizingFieldSettings3D{});

        mesh_ = mesher.mesh();
    }

    static void TearDownTestSuite()
    {
        converter_.reset();
    }

    static TopoDS_Shape shape_;
    static std::unique_ptr<Readers::TopoDS_ShapeConverter> converter_;
    static VolumeMesh3D mesh_;
};

TopoDS_Shape VolumeMesher3DBoxWithHoleTest::shape_;
std::unique_ptr<Readers::TopoDS_ShapeConverter> VolumeMesher3DBoxWithHoleTest::converter_;
VolumeMesh3D VolumeMesher3DBoxWithHoleTest::mesh_;

// ============================================================================
// 1. The mesh is non-empty
// ============================================================================

TEST_F(VolumeMesher3DBoxWithHoleTest, ProducesTetrahedra)
{
    EXPECT_FALSE(mesh_.nodes.empty());
    EXPECT_FALSE(mesh_.boundaryTriangles.empty());
    EXPECT_FALSE(mesh_.tetrahedra.empty())
        << "The volume mesh is empty. A boundary leak lets the ambient flood fill "
           "reach the solid region, so every tetrahedron is classified ambient — "
           "see the watertightness test for the offending edges.";
}

// ============================================================================
// 2. The boundary is watertight
// ============================================================================

TEST_F(VolumeMesher3DBoxWithHoleTest, BoundaryIsWatertight)
{
    // A closed surface is a 2-manifold: every edge separates exactly two
    // triangles. Multiplicity 1 is a hole (a triangle the mesher failed to
    // accept), multiplicity 3 or more is over-acceptance (a triangle it
    // accepted that should not be there). Both break the flood fill.
    std::map<std::array<size_t, 2>, size_t> trianglesPerEdge;
    for (const auto& triangle : mesh_.boundaryTriangles)
    {
        for (const auto& [first, second] : {std::pair{0, 1}, std::pair{1, 2}, std::pair{0, 2}})
        {
            std::array<size_t, 2> edge = {triangle[first], triangle[second]};
            if (edge[0] > edge[1])
                std::swap(edge[0], edge[1]);
            ++trianglesPerEdge[edge];
        }
    }

    for (const auto& [edge, triangleCount] : trianglesPerEdge)
    {
        if (triangleCount == 2)
            continue;

        const Point3D& start = mesh_.nodes[edge[0]];
        const Point3D& end = mesh_.nodes[edge[1]];
        const Point3D midpoint = 0.5 * (start + end);
        ADD_FAILURE() << "Boundary edge (" << edge[0] << ", " << edge[1] << ") is shared by "
                      << triangleCount << " triangles, expected 2. Midpoint ("
                      << midpoint.x() << ", " << midpoint.y() << ", " << midpoint.z()
                      << "), length " << (end - start).norm();
    }
}

// ============================================================================
// 3. The volume matches the analytic value
// ============================================================================

TEST_F(VolumeMesher3DBoxWithHoleTest, VolumeMatchesAnalyticValue)
{
    double meshedVolume = 0.0;
    for (const auto& tetrahedron : mesh_.tetrahedra)
    {
        const Point3D& a = mesh_.nodes[tetrahedron[0]];
        const Point3D& b = mesh_.nodes[tetrahedron[1]];
        const Point3D& c = mesh_.nodes[tetrahedron[2]];
        const Point3D& d = mesh_.nodes[tetrahedron[3]];
        meshedVolume += std::abs((b - a).cross(c - a).dot(d - a)) / 6.0;
    }

    // The bore is meshed as a chordal polygon inscribed in the true circle, so
    // it removes slightly less material than the analytic cylinder does and the
    // meshed volume sits a little above the analytic one — never below it, by
    // more than rounding. OPE-170 measured 880.06 against 874.34 at this
    // discretization. The band below admits that without admitting a mesh that
    // has filled the hole in (which would land near the full 1000).
    const double analytic = analyticVolume();
    EXPECT_GT(meshedVolume, 0.98 * analytic);
    EXPECT_LT(meshedVolume, 1.04 * analytic);
}

// ============================================================================
// 4. Every boundary face group carries triangles
// ============================================================================

TEST_F(VolumeMesher3DBoxWithHoleTest, AllBoundaryFacesCovered)
{
    EXPECT_FALSE(mesh_.boundaryFaceTriangleIds.empty());
    for (const auto& [surfaceId, triangleIds] : mesh_.boundaryFaceTriangleIds)
        EXPECT_FALSE(triangleIds.empty()) << "Boundary face " << surfaceId << " has no triangles";
}
