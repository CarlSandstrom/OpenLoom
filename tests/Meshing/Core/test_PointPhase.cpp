#include "Meshing/Core/3D/RCDT/PointPhase.h"

#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Readers/OpenCascade/TopoDS_ShapeConverter.h"

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>

#include <memory>

#include <gtest/gtest.h>

using namespace Meshing;

// ============================================================================
// CentroidPhaseTest
//
// Pins the reason a tetrahedron's centroid is not a reliable stand-in for the
// tetrahedron when deciding which side of the model it lies on.
//
// RCDT's restriction classifier asks this question through
// DualEdgeRestrictionOracle::isPhaseBoundaryFace(): a face is on the boundary
// when its two adjacent tetrahedra classify to different sides. Centroids are
// used rather than circumcenters because a centroid is a convex combination of
// its own tetrahedron's vertices and so can never escape the element, unlike a
// sliver's circumcenter.
//
// That reasoning is sound about the ELEMENT and unsound about the SOLID. Being
// inside the tetrahedron only makes the centroid representative while the solid
// is locally convex. Across a re-entrant feature a tetrahedron whose four
// vertices all lie on the boundary spans the void, and its centroid lands
// outside the material -- so both tetrahedra sharing a genuine boundary face
// can classify Exterior and the face reads as interior.
//
// Measured on SaddleSurfaceMesh (OPE-186): 19 faces where both adjacent
// tetrahedra were definitive and on the same side, on tetrahedra of median
// volume 12x the mesh median, clustered on the |x| = 2 creases where OPE-187's
// punctures sit. This test is the minimal geometry that reproduces the
// mechanism, so that a replacement oracle can be checked against it directly
// rather than against a 90-second saddle run.
//
// It is a characterization test: it pins the current, wrong-in-context answer.
// A replacement that decides restriction from vertices and surface geometry
// rather than from a single interior sample point should make the failure it
// describes unreachable -- at which point this test documents why the centroid
// alone was not enough.
// ============================================================================

namespace
{

/// An L-shaped prism: the unit-ish box [0,2] x [0,2] x [0,1] with the quadrant
/// x > 1, y > 1 removed. The edge at x = 1, y = 1 is re-entrant, which is the
/// only feature the mechanism needs -- a locally non-convex boundary.
TopoDS_Shape makeReentrantPrism()
{
    const TopoDS_Shape block = BRepPrimAPI_MakeBox(gp_Pnt(0.0, 0.0, 0.0), gp_Pnt(2.0, 2.0, 1.0)).Shape();
    const TopoDS_Shape notch = BRepPrimAPI_MakeBox(gp_Pnt(1.0, 1.0, -0.5), gp_Pnt(3.0, 3.0, 1.5)).Shape();
    return BRepAlgoAPI_Cut(block, notch).Shape();
}

class CentroidPhaseTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        shape_ = makeReentrantPrism();
        converter_ = std::make_unique<Readers::TopoDS_ShapeConverter>(shape_);
    }

    static void TearDownTestSuite() { converter_.reset(); }

    static PointPhase phaseOf(double x, double y, double z)
    {
        return classifyPointPhase(Point3D(x, y, z),
                                  converter_->getTopology().getAllVolumeIds(),
                                  converter_->getGeometryCollection());
    }

    static TopoDS_Shape shape_;
    static std::unique_ptr<Readers::TopoDS_ShapeConverter> converter_;
};

TopoDS_Shape CentroidPhaseTest::shape_;
std::unique_ptr<Readers::TopoDS_ShapeConverter> CentroidPhaseTest::converter_;

} // namespace

// The four points below are the vertices of a tetrahedron spanning the
// re-entrant corner: two on the x = 1 wall, two on the y = 1 wall. Every one of
// them is a legitimate boundary sample of the kind RCDT's discretization and
// refinement both produce.
TEST_F(CentroidPhaseTest, TetrahedronVerticesAreAllOnTheBoundary)
{
    for (const auto& vertex : {Point3D(1.0, 1.5, 0.2),
                               Point3D(1.0, 1.8, 0.8),
                               Point3D(1.5, 1.0, 0.2),
                               Point3D(1.8, 1.0, 0.8)})
    {
        const PointPhase phase = phaseOf(vertex.x(), vertex.y(), vertex.z());
        EXPECT_EQ(phase.kind, PointPhaseKind::Ambiguous)
            << "vertex (" << vertex.x() << ", " << vertex.y() << ", " << vertex.z()
            << ") should classify as on the boundary";
    }
}

// The centroid of those four boundary points. It is not near the boundary and
// not ambiguous -- it is confidently, and for the purpose it is put to,
// misleadingly Exterior.
TEST_F(CentroidPhaseTest, CentroidOfABoundaryTetrahedronCanBeOutsideTheSolid)
{
    const PointPhase centroid = phaseOf(1.325, 1.325, 0.5);

    EXPECT_EQ(centroid.kind, PointPhaseKind::Exterior)
        << "the centroid of four boundary points falls in the removed quadrant";

    // Not a near-boundary case that a tolerance could rescue: the centroid sits
    // 0.325 from either wall, against a classification tolerance of ~3e-4 on a
    // solid this size. Tightening the tolerance cannot change this answer, and
    // OPE-186 measured that tightening it makes the mesh worse anyway.
    EXPECT_NE(centroid.kind, PointPhaseKind::Ambiguous);
}

// Both tetrahedra sharing a face across the re-entrant corner classify the
// same way, which is what makes the failure silent: isPhaseBoundaryFace()
// concludes "not a boundary face" rather than declining.
TEST_F(CentroidPhaseTest, BothSidesOfAReentrantFeatureClassifyExterior)
{
    // Mirror of the first centroid across the re-entrant edge, still spanning
    // the void from the other side.
    EXPECT_EQ(phaseOf(1.325, 1.325, 0.5).kind, PointPhaseKind::Exterior);
    EXPECT_EQ(phaseOf(1.45, 1.45, 0.5).kind, PointPhaseKind::Exterior);

    // For contrast: a point genuinely inside the material classifies correctly,
    // so this is specific to spanning the concavity and not a broken setup.
    EXPECT_EQ(phaseOf(0.5, 0.5, 0.5).kind, PointPhaseKind::InVolume);
}
