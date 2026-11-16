#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Meshing/Data/3D/TetrahedronElement.h"
#include "Meshing/Data/Base/MeshConnectivity.h"

#include <algorithm>
#include <array>
#include <gtest/gtest.h>
#include <memory>

using namespace Meshing;

namespace
{

/// Two tetrahedra sharing the face (node1, node2, node3).
class MeshConnectivityTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        nodeIds_ = {mutator_.addNode(Point3D(0.0, 0.0, 0.0)), mutator_.addNode(Point3D(1.0, 0.0, 0.0)),
                    mutator_.addNode(Point3D(0.0, 1.0, 0.0)), mutator_.addNode(Point3D(0.0, 0.0, 1.0)),
                    mutator_.addNode(Point3D(1.0, 1.0, 1.0))};
        lower_ = addTetrahedron({nodeIds_[0], nodeIds_[1], nodeIds_[2], nodeIds_[3]});
    }

    ElementId addTetrahedron(const std::array<NodeId, 4>& nodeIds)
    {
        return mutator_.addElement(std::make_unique<TetrahedronElement>(nodeIds));
    }

    FaceKey sharedFace() const { return FaceKey(std::array<NodeId, 3>{nodeIds_[1], nodeIds_[2], nodeIds_[3]}); }

    MeshData3D meshData_;
    MeshMutator3D mutator_{meshData_};
    std::array<NodeId, 5> nodeIds_;
    ElementId lower_;
};

TEST_F(MeshConnectivityTest, AddElementRecordsItsNodesAndFaces)
{
    MeshConnectivity connectivity(meshData_);
    auto upper = addTetrahedron({nodeIds_[1], nodeIds_[2], nodeIds_[3], nodeIds_[4]});

    connectivity.addElement(upper);

    EXPECT_EQ(connectivity.getFaceElements(sharedFace()), std::make_pair(lower_, upper));
    EXPECT_EQ(connectivity.getNodeElements(nodeIds_[4]), std::vector<ElementId>{upper});
    EXPECT_EQ(connectivity.getNodeElements(nodeIds_[1]), (std::vector<ElementId>{lower_, upper}));
}

TEST_F(MeshConnectivityTest, RemoveElementLeavesTheOtherElementOnASharedFace)
{
    auto upper = addTetrahedron({nodeIds_[1], nodeIds_[2], nodeIds_[3], nodeIds_[4]});
    MeshConnectivity connectivity(meshData_);

    connectivity.removeElement(lower_);

    EXPECT_EQ(connectivity.getFaceElements(sharedFace()), std::make_pair(upper, ElementId::INVALID));
    EXPECT_EQ(connectivity.getNodeElements(nodeIds_[1]), std::vector<ElementId>{upper});
    EXPECT_TRUE(connectivity.canRemoveNode(nodeIds_[0]));
    EXPECT_FALSE(connectivity.canRemoveNode(nodeIds_[4]));
}

TEST_F(MeshConnectivityTest, RemoveElementForgetsAFaceWithNoElementLeft)
{
    MeshConnectivity connectivity(meshData_);

    connectivity.removeElement(lower_);

    EXPECT_EQ(connectivity.getFaceElements(sharedFace()).first, ElementId::INVALID);
    EXPECT_TRUE(connectivity.canRemoveNode(nodeIds_[1]));
}

} // namespace
