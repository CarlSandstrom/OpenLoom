#include "Meshing/Core/3D/RCDT/BoundaryDefectReporter.h"

#include "Common/Exceptions/MeshException.h"
#include "Common/Types.h"
#include "Meshing/Connectivity/FaceKey.h"
#include "Meshing/Core/3D/RCDT/RestrictedFaceAudit.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"

#include <gtest/gtest.h>

using namespace Meshing;

// ============================================================================
// BoundaryDefectReporterTest
//
// The audit names edges by working-mesh node IDs; the report the caller gets
// carries their coordinates and, when a mesh was produced, their IDs in that
// mesh's numbering.
// ============================================================================

namespace
{

constexpr const char* SURFACE_ID = "surface";

// A unit square on z = 0 split along its diagonal: the diagonal carries two
// faces, the 4 outer edges one each, so the audit reports those 4 as missing
// a face.
struct OpenSquare
{
    MeshData3D meshData;
    NodeId n0{0};
    NodeId n1{0};
    NodeId n2{0};
    NodeId n3{0};
    std::vector<NonManifoldRestrictedEdge> defects;

    OpenSquare()
    {
        MeshMutator3D mutator(meshData);
        n0 = mutator.addBoundaryNode(Point3D(0.0, 0.0, 0.0), {SURFACE_ID});
        n1 = mutator.addBoundaryNode(Point3D(1.0, 0.0, 0.0), {SURFACE_ID});
        n2 = mutator.addBoundaryNode(Point3D(1.0, 1.0, 0.0), {SURFACE_ID});
        n3 = mutator.addBoundaryNode(Point3D(0.0, 1.0, 0.0), {SURFACE_ID});
        RestrictedFaceMap faces{{FaceKey(n0, n1, n2), SURFACE_ID}, {FaceKey(n0, n2, n3), SURFACE_ID}};
        defects = RestrictedFaceAudit::findNonManifoldEdges(faces, {}, meshData);
    }

    Point3D coordinatesOf(NodeId nodeId) const { return meshData.getNode(nodeId)->getCoordinates(); }
};

// Reverses the IDs of 4 nodes numbered 0..3, so a translated ID can't pass by
// accident.
std::vector<NodeId> reversedRenumbering()
{
    return {NodeId{3}, NodeId{2}, NodeId{1}, NodeId{0}};
}

} // namespace

TEST(BoundaryDefectReporterTest, ReportsEveryAuditedEdgeWithItsEndpointCoordinates)
{
    OpenSquare square;

    auto report = BoundaryDefectReporter::toReport(square.defects, square.meshData);

    ASSERT_EQ(report.defects.size(), 4u);
    for (size_t index = 0; index < report.defects.size(); ++index)
    {
        auto& entry = report.defects[index];
        auto& edge = square.defects[index].edge;
        EXPECT_EQ(entry.kind, BoundaryDefectKind3D::MissingFace);
        EXPECT_EQ(entry.surfaceId, SURFACE_ID);
        EXPECT_EQ(entry.endpoints[0], square.coordinatesOf(edge.nodeIds[0]));
        EXPECT_EQ(entry.endpoints[1], square.coordinatesOf(edge.nodeIds[1]));
        EXPECT_FALSE(entry.nodeIds.has_value());
    }
}

TEST(BoundaryDefectReporterTest, TranslatesNodeIdsThroughTheRenumbering)
{
    OpenSquare square;
    auto renumbering = reversedRenumbering();

    auto report = BoundaryDefectReporter::toReport(square.defects, square.meshData, renumbering);

    ASSERT_EQ(report.defects.size(), 4u);
    for (size_t index = 0; index < report.defects.size(); ++index)
    {
        auto& edge = square.defects[index].edge;
        ASSERT_TRUE(report.defects[index].nodeIds.has_value());
        EXPECT_EQ((*report.defects[index].nodeIds)[0], renumbering[edge.nodeIds[0].value()]);
        EXPECT_EQ((*report.defects[index].nodeIds)[1], renumbering[edge.nodeIds[1].value()]);
    }
}

TEST(BoundaryDefectReporterTest, ThrowsWhenADefectNodeIsNotInTheOutputMesh)
{
    OpenSquare square;
    auto renumbering = reversedRenumbering();
    renumbering[square.n1.value()] = NodeId::INVALID;

    EXPECT_THROW(BoundaryDefectReporter::toReport(square.defects, square.meshData, renumbering),
                 OpenLoom::MeshException);
}
