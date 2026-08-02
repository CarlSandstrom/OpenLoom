#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/Surface/SurfaceMesher3D.h"
#include "Meshing/Data/3D/SurfaceMesh3D.h"
#include "Readers/OpenCascade/TopoDS_ShapeConverter.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <numbers>

#include <gtest/gtest.h>

using namespace Meshing;

// ============================================================================
// SurfaceMesher3DStrategyTest
//
// Covers OPE-137/OPE-139: Auto must route a non-periodic shape (box, no
// seams) to PerFaceUV and a periodic shape (cylinder, has seams) to
// AmbientRCDT, and an explicit strategy must always be honored regardless
// of topology.
// ============================================================================

TEST(SurfaceMesher3DStrategyTest, AutoResolvesToPerFaceUVForNonPeriodicShape)
{
    const TopoDS_Shape shape = BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape();
    Readers::TopoDS_ShapeConverter converter(shape);

    const Geometry3D::DiscretizationSettings3D discretizationSettings(2, 0);

    SurfaceMesher3D mesher(converter.getGeometryCollection(),
                           converter.getTopology(),
                           discretizationSettings);

    EXPECT_EQ(mesher.getStrategy(), SurfaceMeshingStrategy::PerFaceUV);

    const SurfaceMesh3D mesh = mesher.mesh();
    EXPECT_FALSE(mesh.triangles.empty());
}

TEST(SurfaceMesher3DStrategyTest, AutoResolvesToAmbientRCDTForPeriodicShape)
{
    const gp_Pnt origin(0.0, 0.0, 0.0);
    const gp_Dir axisZ(0.0, 0.0, 1.0);
    const TopoDS_Shape shape = BRepPrimAPI_MakeCylinder(gp_Ax2(origin, axisZ), 3.0, 8.0).Shape();
    Readers::TopoDS_ShapeConverter converter(shape);

    const Geometry3D::DiscretizationSettings3D discretizationSettings(
        std::nullopt, std::numbers::pi * 2.0 / 20.0 + 0.01, 0);

    SurfaceMesher3D mesher(converter.getGeometryCollection(),
                           converter.getTopology(),
                           discretizationSettings);

    EXPECT_EQ(mesher.getStrategy(), SurfaceMeshingStrategy::AmbientRCDT);

    const SurfaceMesh3D mesh = mesher.mesh();
    EXPECT_FALSE(mesh.triangles.empty());
}

TEST(SurfaceMesher3DStrategyTest, ExplicitStrategyOverridesAuto)
{
    // A periodic shape: Auto alone would resolve this to AmbientRCDT (see
    // AutoResolvesToAmbientRCDTForPeriodicShape above). Requesting PerFaceUV
    // explicitly must still be honored despite the seam.
    const gp_Pnt origin(0.0, 0.0, 0.0);
    const gp_Dir axisZ(0.0, 0.0, 1.0);
    const TopoDS_Shape shape = BRepPrimAPI_MakeCylinder(gp_Ax2(origin, axisZ), 3.0, 8.0).Shape();
    Readers::TopoDS_ShapeConverter converter(shape);

    const Geometry3D::DiscretizationSettings3D discretizationSettings(
        std::nullopt, std::numbers::pi * 2.0 / 20.0 + 0.01, 0);

    SurfaceMesher3D mesher(converter.getGeometryCollection(),
                           converter.getTopology(),
                           discretizationSettings,
                           SurfaceMesh3DQualitySettings{},
                           SurfaceMeshingStrategy::PerFaceUV);

    EXPECT_EQ(mesher.getStrategy(), SurfaceMeshingStrategy::PerFaceUV);
}
