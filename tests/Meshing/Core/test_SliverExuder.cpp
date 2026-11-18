#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/General/ElementGeometry3D.h"
#include "Meshing/Core/3D/Volume/VolumeMesher3D.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"
#include "Meshing/Data/3D/VolumeMesh3D.h"
#include "Readers/OpenCascade/TopoDS_ShapeConverter.h"

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <array>
#include <numbers>
#include <set>

#include <gtest/gtest.h>

using namespace Meshing;

namespace
{

/// The ThinCapBoxVolume example's model: a 10 mm box with a pocket cut from
/// below, leaving a 1 mm cap. Without exudation its cap is full of slivers,
/// down to 0 degrees.
VolumeMesh3D meshThinCapBox(double sliverDihedralAngleDegrees)
{
    TopoDS_Shape box = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();
    TopoDS_Shape pocket = BRepPrimAPI_MakeBox(gp_Pnt(2.0, 2.0, 0.0), 6.0, 6.0, 9.0).Shape();
    Readers::TopoDS_ShapeConverter converter(BRepAlgoAPI_Cut(box, pocket).Shape());

    SurfaceMesh3DQualitySettings quality;
    quality.sliverDihedralAngleDegrees = sliverDihedralAngleDegrees;
    // Smoothing moves boundary nodes, so with it off both runs keep the
    // refined positions and differ only by exudation.
    quality.smoothingIterations = 0;

    VolumeMesher3D mesher(converter.getGeometryCollection(), converter.getTopology(),
                          Geometry3D::DiscretizationSettings3D(std::nullopt, std::numbers::pi / 8.0, 2), quality);
    mesher.mesh();
    return *mesher.getMesh();
}

std::set<std::array<NodeId, 3>> sortedTriangles(const VolumeMesh3D& mesh)
{
    std::set<std::array<NodeId, 3>> triangles;
    for (auto triangle : mesh.boundaryTriangles)
    {
        std::sort(triangle.begin(), triangle.end());
        triangles.insert(triangle);
    }
    return triangles;
}

double worstDihedralAngle(const VolumeMesh3D& mesh)
{
    double worst = 180.0;
    for (auto& tetrahedron : mesh.tetrahedra)
        worst = std::min(worst, ElementGeometry3D::computeMinimumDihedralAngle(
                                    {mesh.nodes[tetrahedron[0].value()], mesh.nodes[tetrahedron[1].value()],
                                     mesh.nodes[tetrahedron[2].value()], mesh.nodes[tetrahedron[3].value()]}));
    return worst;
}

} // namespace

// Exudation changes weights, never positions, and undoes any change that
// alters a restricted face: the boundary mesh must come out exactly as
// refinement left it, while the slivers inside the thin cap improve.
TEST(SliverExuderTest, ThinCapSliversImproveWithoutTouchingTheBoundary)
{
    auto plain = meshThinCapBox(0.0);
    auto exuded = meshThinCapBox(15.0);

    EXPECT_EQ(exuded.nodes, plain.nodes);
    EXPECT_EQ(sortedTriangles(exuded), sortedTriangles(plain));

    EXPECT_LT(worstDihedralAngle(plain), 1.0);
    EXPECT_GT(worstDihedralAngle(exuded), 5.0);
}
