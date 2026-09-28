#include "Meshing/Core/3D/RCDT/RestrictedFaceAudit.h"

#include "Common/Types.h"
#include "Meshing/Connectivity/EdgeKey.h"
#include "Meshing/Connectivity/FaceKey.h"
#include "Meshing/Core/3D/RCDT/RestrictedFaceTypes.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"

#include <gtest/gtest.h>

using namespace Meshing;

// ============================================================================
// RestrictedFaceAuditTest -- findNonManifoldEdges
//
// RCDTMesher::meshVolume() refuses a restricted boundary with holes by
// counting the MissingFace edges this reports. On a surface interior (no model
// curve), an edge needs exactly two restricted faces.
// ============================================================================

namespace
{

constexpr const char* SURFACE_ID = "surface";

// A unit square on z = 0, every node on SURFACE_ID and none on a curve.
struct SquareSetup
{
    MeshData3D meshData;
    size_t n0 = 0;
    size_t n1 = 0;
    size_t n2 = 0;
    size_t n3 = 0;

    SquareSetup()
    {
        MeshMutator3D mutator(meshData);
        n0 = mutator.addBoundaryNode(Point3D(0.0, 0.0, 0.0), {SURFACE_ID});
        n1 = mutator.addBoundaryNode(Point3D(1.0, 0.0, 0.0), {SURFACE_ID});
        n2 = mutator.addBoundaryNode(Point3D(1.0, 1.0, 0.0), {SURFACE_ID});
        n3 = mutator.addBoundaryNode(Point3D(0.0, 1.0, 0.0), {SURFACE_ID});
    }
};

} // namespace

// A single triangle isn't closed: all 3 of its edges carry one face, not two.
TEST(RestrictedFaceAuditTest, FindNonManifoldEdges_SingleFace_AllThreeEdgesAreMissingAFace)
{
    const SquareSetup setup;
    const RestrictedFaceMap faces{{FaceKey(setup.n0, setup.n1, setup.n2), SURFACE_ID}};

    const auto defects = RestrictedFaceAudit::findNonManifoldEdges(faces, {}, setup.meshData);

    ASSERT_EQ(defects.size(), 3u);
    for (const auto& defect : defects)
    {
        EXPECT_EQ(defect.surfaceId, SURFACE_ID);
        EXPECT_EQ(defect.defect, RestrictedEdgeDefect::MissingFace);
    }
}

// The square split along its diagonal: the shared diagonal carries two faces
// and is not reported; the 4 outer edges carry one each and are.
TEST(RestrictedFaceAuditTest, FindNonManifoldEdges_SharedEdgeNotReported_OpenBoundaryIs)
{
    const SquareSetup setup;
    const RestrictedFaceMap faces{{FaceKey(setup.n0, setup.n1, setup.n2), SURFACE_ID},
                                  {FaceKey(setup.n0, setup.n2, setup.n3), SURFACE_ID}};

    const auto defects = RestrictedFaceAudit::findNonManifoldEdges(faces, {}, setup.meshData);

    ASSERT_EQ(defects.size(), 4u);
    for (const auto& defect : defects)
        EXPECT_FALSE(defect.edge == EdgeKey(setup.n0, setup.n2));
}
