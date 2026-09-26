#include "Meshing/Core/3D/RCDT/RestrictedTriangulation.h"

#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Connectivity/FaceKey.h"
#include "Meshing/Core/3D/RCDT/PointPhase.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"
#include "Meshing/Data/3D/TetrahedralElement.h"
#include "Meshing/Data/Base/MeshConnectivity.h"
#include "Readers/OpenCascade/TopoDS_ShapeConverter.h"
#include "Topology/Topology3D.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>

#include <array>
#include <limits>
#include <memory>
#include <string>

#include <gtest/gtest.h>

using namespace Meshing;

// ============================================================================
// PhaseBoundaryFaceTest
//
// The defect itself, as opposed to its premise.
//
// A face of the tetrahedralization lies exactly on the model boundary, and the
// classifier fails to recognise it, because BOTH tetrahedra sharing it have
// centroids outside the solid. That is possible whenever the material is
// thinner than the elements spanning it: a tetrahedron can contain a slab of
// material and still have its centroid clear of it.
//
// Reproduced here on a plate 0.1 thick with tetrahedra ~0.5 across -- no
// refinement, no curvature, no CAD complexity beyond a box.
// ============================================================================

namespace
{

constexpr double PLATE_THICKNESS = 0.1;

TopoDS_Shape makeThinPlate()
{
    return BRepPrimAPI_MakeBox(gp_Pnt(0.0, 0.0, 0.0), gp_Pnt(2.0, 2.0, PLATE_THICKNESS)).Shape();
}

/// The surface id whose geometry passes closest to probe -- used to find which
/// of the box's six faces is the top one.
std::string surfaceNearest(const Point3D& probe,
                           const Topology3D::Topology3D& topology,
                           const Geometry3D::GeometryCollection3D& geometry)
{
    std::string best;
    double bestGap = std::numeric_limits<double>::max();
    for (const auto& surfaceId : topology.getAllSurfaceIds())
    {
        const Geometry3D::ISurface3D* surface = geometry.getSurface(surfaceId);
        if (!surface)
            continue;
        const double gap = surface->getGap(probe);
        if (gap < bestGap)
        {
            bestGap = gap;
            best = surfaceId;
        }
    }
    return best;
}

class PhaseBoundaryFaceTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        shape_ = makeThinPlate();
        converter_ = std::make_unique<Readers::TopoDS_ShapeConverter>(shape_);
        topSurfaceId_ = surfaceNearest(Point3D(1.0, 1.0, PLATE_THICKNESS),
                                       converter_->getTopology(),
                                       converter_->getGeometryCollection());
    }

    static void TearDownTestSuite() { converter_.reset(); }

    static PointPhase phaseOf(const Point3D& point)
    {
        return classifyPointPhase(point,
                                  converter_->getTopology().getAllVolumeIds(),
                                  converter_->getGeometryCollection());
    }

    static TopoDS_Shape shape_;
    static std::unique_ptr<Readers::TopoDS_ShapeConverter> converter_;
    static std::string topSurfaceId_;
};

TopoDS_Shape PhaseBoundaryFaceTest::shape_;
std::unique_ptr<Readers::TopoDS_ShapeConverter> PhaseBoundaryFaceTest::converter_;
std::string PhaseBoundaryFaceTest::topSurfaceId_;

/// Three boundary nodes on the plate's top face, plus one apex above and one
/// below. The lower tetrahedron pierces the whole plate and emerges underneath.
struct ThinPlateSetup
{
    MeshData3D meshData;
    size_t n0 = 0;
    size_t n1 = 0;
    size_t n2 = 0;
    size_t above = 0;
    size_t below = 0;

    explicit ThinPlateSetup(const std::string& surfaceId)
    {
        MeshMutator3D mutator(meshData);
        n0 = mutator.addBoundaryNode(Point3D(0.5, 0.5, PLATE_THICKNESS), {surfaceId});
        n1 = mutator.addBoundaryNode(Point3D(1.5, 0.5, PLATE_THICKNESS), {surfaceId});
        n2 = mutator.addBoundaryNode(Point3D(1.0, 1.5, PLATE_THICKNESS), {surfaceId});
        above = mutator.addNode(Point3D(1.0, 0.8, PLATE_THICKNESS + 0.5));
        below = mutator.addNode(Point3D(1.0, 0.8, -0.5));

        mutator.addElement(std::make_unique<TetrahedralElement>(std::array<size_t, 4>{n0, n1, n2, above}));
        mutator.addElement(std::make_unique<TetrahedralElement>(std::array<size_t, 4>{n0, n1, n2, below}));
    }
};

} // namespace

// The setup is what it claims: the shared face lies on the top surface, and
// both tetrahedra's centroids are outside the solid -- the lower one despite
// containing a slab of the plate.
TEST_F(PhaseBoundaryFaceTest, BothCentroidsAreOutsideThoughTheFaceIsOnTheBoundary)
{
    ASSERT_FALSE(topSurfaceId_.empty());

    const Point3D upperCentroid(1.0, 0.825, (3.0 * PLATE_THICKNESS + PLATE_THICKNESS + 0.5) / 4.0);
    const Point3D lowerCentroid(1.0, 0.825, (3.0 * PLATE_THICKNESS - 0.5) / 4.0);

    EXPECT_EQ(phaseOf(upperCentroid).kind, PointPhaseKind::Exterior);
    EXPECT_EQ(phaseOf(lowerCentroid).kind, PointPhaseKind::Exterior);

    // The three shared vertices really are on the boundary.
    for (const auto& vertex : {Point3D(0.5, 0.5, PLATE_THICKNESS),
                               Point3D(1.5, 0.5, PLATE_THICKNESS),
                               Point3D(1.0, 1.5, PLATE_THICKNESS)})
    {
        EXPECT_EQ(phaseOf(vertex).kind, PointPhaseKind::Ambiguous);
    }
}

// THE DEFECT. The shared face lies on the model boundary and must be
// restricted to the top surface. The classifier does not recognise it.
TEST_F(PhaseBoundaryFaceTest, AFaceOnTheBoundaryOfAThinPlateIsRestricted)
{
    ASSERT_FALSE(topSurfaceId_.empty());

    ThinPlateSetup setup(topSurfaceId_);
    const MeshConnectivity connectivity(setup.meshData);

    RestrictedTriangulation restrictedTriangulation;
    restrictedTriangulation.buildFrom(setup.meshData,
                                      connectivity,
                                      converter_->getGeometryCollection(),
                                      converter_->getTopology(),
                                      0.05,
                                      SurfaceMesh3DQualitySettings{});

    const FaceKey sharedFace(std::array<size_t, 3>{setup.n0, setup.n1, setup.n2});
    const auto& restricted = restrictedTriangulation.getRestrictedFaces();

    EXPECT_TRUE(restricted.count(sharedFace))
        << "the face lies exactly on the plate's top surface and is shared by the only two "
           "tetrahedra, so it is the boundary there -- but both centroids classify Exterior, "
           "so the phase test reports 'same side' rather than declining, and nothing else "
           "recovers it";
}
