#include <gtest/gtest.h>

#include "Meshing/Core/3D/General/UnreferencedNodeRemover.h"
#include "Meshing/Data/3D/SurfaceMesh3D.h"
#include "Meshing/Data/3D/VolumeMesh3D.h"

using namespace Meshing;

namespace
{

// Node i sits at (i, 0, 0), so a node's original ID can be read off its position.
std::vector<Point3D> numberedNodes(size_t count)
{
    std::vector<Point3D> nodes;
    for (size_t index = 0; index < count; ++index)
        nodes.emplace_back(static_cast<double>(index), 0.0, 0.0);
    return nodes;
}

} // namespace

TEST(UnreferencedNodeRemoverTest, SurfaceMeshKeepsReferencedNodesInOrderAndRenumbers)
{
    SurfaceMesh3D mesh;
    mesh.nodes = numberedNodes(8);
    mesh.triangles = {{NodeId{4}, NodeId{6}, NodeId{5}}};
    mesh.faceTriangleIds["face_0"] = {0};
    mesh.edgeNodeIds["edge_0"] = {NodeId{7}, NodeId{4}};

    UnreferencedNodeRemover::remove(mesh);

    ASSERT_EQ(mesh.nodes.size(), 4u);
    EXPECT_DOUBLE_EQ(mesh.nodes[0].x(), 4.0);
    EXPECT_DOUBLE_EQ(mesh.nodes[1].x(), 5.0);
    EXPECT_DOUBLE_EQ(mesh.nodes[2].x(), 6.0);
    EXPECT_DOUBLE_EQ(mesh.nodes[3].x(), 7.0);
    EXPECT_EQ(mesh.triangles[0], (std::array<NodeId, 3>{NodeId{0}, NodeId{2}, NodeId{1}}));
    EXPECT_EQ(mesh.edgeNodeIds.at("edge_0"), (std::vector<NodeId>{NodeId{3}, NodeId{0}}));
    EXPECT_EQ(mesh.faceTriangleIds.at("face_0"), (std::vector<size_t>{0}));
}

TEST(UnreferencedNodeRemoverTest, ReturnsTheRenumberingFromOldToNewNodeIds)
{
    SurfaceMesh3D mesh;
    mesh.nodes = numberedNodes(6);
    mesh.triangles = {{NodeId{1}, NodeId{3}, NodeId{4}}};

    auto renumbering = UnreferencedNodeRemover::remove(mesh);

    EXPECT_EQ(renumbering, (std::vector<NodeId>{NodeId::INVALID, NodeId{0}, NodeId::INVALID, NodeId{1}, NodeId{2},
                                                NodeId::INVALID}));
}

TEST(UnreferencedNodeRemoverTest, VolumeMeshKeepsInteriorNodesOnlyATetrahedronUses)
{
    VolumeMesh3D mesh;
    mesh.nodes = numberedNodes(9);
    mesh.tetrahedra = {{NodeId{4}, NodeId{5}, NodeId{6}, NodeId{8}}};
    mesh.boundaryTriangles = {{NodeId{4}, NodeId{5}, NodeId{6}}};
    mesh.boundaryEdgeNodeIds["edge_0"] = {NodeId{4}, NodeId{5}};

    UnreferencedNodeRemover::remove(mesh);

    ASSERT_EQ(mesh.nodes.size(), 4u);
    EXPECT_DOUBLE_EQ(mesh.nodes[3].x(), 8.0);
    EXPECT_EQ(mesh.tetrahedra[0], (std::array<NodeId, 4>{NodeId{0}, NodeId{1}, NodeId{2}, NodeId{3}}));
    EXPECT_EQ(mesh.boundaryTriangles[0], (std::array<NodeId, 3>{NodeId{0}, NodeId{1}, NodeId{2}}));
    EXPECT_EQ(mesh.boundaryEdgeNodeIds.at("edge_0"), (std::vector<NodeId>{NodeId{0}, NodeId{1}}));
}
