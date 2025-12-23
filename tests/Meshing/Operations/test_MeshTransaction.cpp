#include "Common/Types.h"
#include "Meshing/Data/MeshConnectivity.h"
#include "Meshing/Data/MeshData3D.h"
#include "Meshing/Data/MeshMutator3D.h"
#include "Meshing/Data/TetrahedralElement.h"
#include "Meshing/Operations/MeshTransaction.h"
#include "Meshing/Operations/ScopedTransaction.h"
#include <array>
#include <gtest/gtest.h>
#include <memory>

using namespace Meshing;

namespace Operations
{

// Helper class to create a mock element for testing
class MockTetrahedralElement : public TetrahedralElement
{
public:
    MockTetrahedralElement(const std::array<size_t, 4>& nodeIds) :
        TetrahedralElement(nodeIds) {}

    std::unique_ptr<IElement> clone() const override
    {
        return std::make_unique<MockTetrahedralElement>(
            std::array<size_t, 4>{getNodeIds()[0], getNodeIds()[1],
                                  getNodeIds()[2], getNodeIds()[3]});
    }
};

class MeshTransactionTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        geometry_ = std::make_unique<MeshData3D>();
        meshMutator_ = std::make_unique<MeshMutator3D>(*geometry_);
        connectivity_ = std::make_unique<MeshConnectivity>(*geometry_);

        // Wire up operations to use connectivity for validation
        meshMutator_->setConnectivity(connectivity_.get());

        // Add some test nodes
        node1Id_ = meshMutator_->addNode(Point3D(0.0, 0.0, 0.0));
        node2Id_ = meshMutator_->addNode(Point3D(1.0, 0.0, 0.0));
        node3Id_ = meshMutator_->addNode(Point3D(0.0, 1.0, 0.0));
        node4Id_ = meshMutator_->addNode(Point3D(0.0, 0.0, 1.0));
        node5Id_ = meshMutator_->addNode(Point3D(1.0, 1.0, 1.0));

        // Rebuild connectivity after adding nodes
        connectivity_->rebuildConnectivity();
    }

    void TearDown() override
    {
        meshMutator_.reset();
        connectivity_.reset();
        geometry_.reset();
    }

    std::unique_ptr<MeshData3D> geometry_;
    std::unique_ptr<MeshMutator3D> meshMutator_;
    std::unique_ptr<MeshConnectivity> connectivity_;
    size_t node1Id_, node2Id_, node3Id_, node4Id_, node5Id_;
};

TEST_F(MeshTransactionTest, ConstructorTest)
{
    MeshTransaction transaction(meshMutator_.get());

    EXPECT_FALSE(transaction.isActive());
}

TEST_F(MeshTransactionTest, BeginTransactionTest)
{
    MeshTransaction transaction(meshMutator_.get());

    EXPECT_FALSE(transaction.isActive());

    transaction.begin();

    EXPECT_TRUE(transaction.isActive());
}

TEST_F(MeshTransactionTest, CommitTransactionTest)
{
    MeshTransaction transaction(meshMutator_.get());
    transaction.begin();

    auto element = std::make_unique<MockTetrahedralElement>(
        std::array<size_t, 4>{node1Id_, node2Id_, node3Id_, node4Id_});

    EXPECT_EQ(geometry_->getElementCount(), 0);
    size_t elementId = meshMutator_->addElement(std::move(element));

    EXPECT_EQ(geometry_->getElementCount(), 1);
    EXPECT_TRUE(transaction.isActive());

    transaction.commit();

    EXPECT_EQ(geometry_->getElementCount(), 1);
    EXPECT_FALSE(transaction.isActive());
}

TEST_F(MeshTransactionTest, RollbackTransactionTest)
{
    MeshTransaction transaction(meshMutator_.get());
    transaction.begin();

    auto element = std::make_unique<MockTetrahedralElement>(
        std::array<size_t, 4>{node1Id_, node2Id_, node3Id_, node4Id_});

    EXPECT_EQ(geometry_->getElementCount(), 0);
    size_t elementId = meshMutator_->addElement(std::move(element));

    EXPECT_TRUE(transaction.isActive());
    EXPECT_EQ(geometry_->getElementCount(), 1);

    transaction.rollback();

    EXPECT_FALSE(transaction.isActive());
    EXPECT_EQ(geometry_->getElementCount(), 0);
}

TEST_F(MeshTransactionTest, AutoRollbackOnDestructorTest)
{
    EXPECT_EQ(geometry_->getElementCount(), 0);

    {
        MeshTransaction transaction(meshMutator_.get());
        transaction.begin();

        // Add an element during transaction
        auto element = std::make_unique<MockTetrahedralElement>(
            std::array<size_t, 4>{node1Id_, node2Id_, node3Id_, node4Id_});
        size_t elementId = meshMutator_->addElement(std::move(element));

        EXPECT_EQ(geometry_->getElementCount(), 1);
        EXPECT_TRUE(transaction.isActive());

        // Transaction should auto-rollback when going out of scope
    }

    // Element should be removed after auto-rollback
    EXPECT_EQ(geometry_->getElementCount(), 0);
}

TEST_F(MeshTransactionTest, NodeAdditionAndRollbackTest)
{
    size_t initialNodeCount = geometry_->getNodeCount();

    MeshTransaction transaction(meshMutator_.get());
    transaction.begin();

    // Add a node during transaction
    size_t newNodeId = meshMutator_->addNode(Point3D(2.0, 2.0, 2.0));

    EXPECT_EQ(geometry_->getNodeCount(), initialNodeCount + 1);
    EXPECT_TRUE(transaction.isActive());

    transaction.rollback();

    EXPECT_FALSE(transaction.isActive());
    // Node should be removed after rollback
    EXPECT_EQ(geometry_->getNodeCount(), initialNodeCount);
}

TEST_F(MeshTransactionTest, NodeModificationAndRollbackTest)
{
    // Get original coordinates
    const Node3D* node = geometry_->getNode(node1Id_);
    ASSERT_NE(node, nullptr);
    Point3D originalCoords = node->getCoordinates();

    MeshTransaction transaction(meshMutator_.get());
    transaction.begin();

    // Modify an existing node during transaction
    Point3D newCoords(1.5, 1.5, 1.5);
    meshMutator_->moveNode(node1Id_, newCoords);

    // Verify the node was modified
    node = geometry_->getNode(node1Id_);
    ASSERT_NE(node, nullptr);
    EXPECT_TRUE(node->getCoordinates().isApprox(newCoords));
    EXPECT_TRUE(transaction.isActive());

    transaction.rollback();

    EXPECT_FALSE(transaction.isActive());
    // Node coordinates should be restored after rollback
    node = geometry_->getNode(node1Id_);
    ASSERT_NE(node, nullptr);
    EXPECT_TRUE(node->getCoordinates().isApprox(originalCoords));
}

TEST_F(MeshTransactionTest, ElementRemovalAndRollbackTest)
{
    // First add an element outside transaction
    auto element = std::make_unique<MockTetrahedralElement>(
        std::array<size_t, 4>{node1Id_, node2Id_, node3Id_, node4Id_});
    size_t elementId = meshMutator_->addElement(std::move(element));

    EXPECT_EQ(geometry_->getElementCount(), 1);
    EXPECT_NE(geometry_->getElement(elementId), nullptr);

    // Now remove it within a transaction
    MeshTransaction transaction(meshMutator_.get());
    transaction.begin();

    meshMutator_->removeElement(elementId);

    EXPECT_EQ(geometry_->getElementCount(), 0);
    EXPECT_EQ(geometry_->getElement(elementId), nullptr);
    EXPECT_TRUE(transaction.isActive());

    transaction.rollback();

    EXPECT_FALSE(transaction.isActive());
    // Element should be restored after rollback
    EXPECT_EQ(geometry_->getElementCount(), 1);
    EXPECT_NE(geometry_->getElement(elementId), nullptr);
}

TEST_F(MeshTransactionTest, MultipleOperationsRollbackTest)
{
    size_t initialNodeCount = geometry_->getNodeCount();
    size_t initialElementCount = geometry_->getElementCount();

    // Get original coordinates of node2
    const Node3D* node2 = geometry_->getNode(node2Id_);
    ASSERT_NE(node2, nullptr);
    Point3D originalNode2Coords = node2->getCoordinates();

    MeshTransaction transaction(meshMutator_.get());
    transaction.begin();

    // Perform multiple operations
    size_t newNodeId = meshMutator_->addNode(Point3D(3.0, 3.0, 3.0));
    meshMutator_->moveNode(node2Id_, Point3D(2.5, 2.5, 2.5));

    auto element = std::make_unique<MockTetrahedralElement>(
        std::array<size_t, 4>{node1Id_, node3Id_, node4Id_, newNodeId});
    size_t elementId = meshMutator_->addElement(std::move(element));

    // Verify operations were applied
    EXPECT_EQ(geometry_->getNodeCount(), initialNodeCount + 1);
    EXPECT_EQ(geometry_->getElementCount(), initialElementCount + 1);
    node2 = geometry_->getNode(node2Id_);
    ASSERT_NE(node2, nullptr);
    EXPECT_FALSE(node2->getCoordinates().isApprox(originalNode2Coords));
    EXPECT_TRUE(transaction.isActive());

    transaction.rollback();

    EXPECT_FALSE(transaction.isActive());
    // All operations should be rolled back
    EXPECT_EQ(geometry_->getNodeCount(), initialNodeCount);
    EXPECT_EQ(geometry_->getElementCount(), initialElementCount);
    node2 = geometry_->getNode(node2Id_);
    ASSERT_NE(node2, nullptr);
    EXPECT_TRUE(node2->getCoordinates().isApprox(originalNode2Coords));
    EXPECT_EQ(geometry_->getNode(newNodeId), nullptr);
    EXPECT_EQ(geometry_->getElement(elementId), nullptr);
}

TEST_F(MeshTransactionTest, NestedTransactionBehaviorTest)
{
    EXPECT_EQ(geometry_->getElementCount(), 0);

    MeshTransaction transaction1(meshMutator_.get());
    transaction1.begin();

    // Add element in first transaction
    auto element1 = std::make_unique<MockTetrahedralElement>(
        std::array<size_t, 4>{node1Id_, node2Id_, node3Id_, node4Id_});
    size_t elementId1 = meshMutator_->addElement(std::move(element1));

    EXPECT_EQ(geometry_->getElementCount(), 1);
    transaction1.commit();
    EXPECT_EQ(geometry_->getElementCount(), 1);

    // Second transaction
    MeshTransaction transaction2(meshMutator_.get());
    transaction2.begin();

    // Add another element
    auto element2 = std::make_unique<MockTetrahedralElement>(
        std::array<size_t, 4>{node2Id_, node3Id_, node4Id_, node5Id_});
    size_t elementId2 = meshMutator_->addElement(std::move(element2));

    EXPECT_EQ(geometry_->getElementCount(), 2);
    transaction2.rollback();

    // First element should remain, second should be rolled back
    EXPECT_EQ(geometry_->getElementCount(), 1);
    EXPECT_NE(geometry_->getElement(elementId1), nullptr);
    EXPECT_EQ(geometry_->getElement(elementId2), nullptr);
}

// Tests for ScopedTransaction
class ScopedTransactionTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        geometry_ = std::make_unique<MeshData3D>();
        meshMutator_ = std::make_unique<MeshMutator3D>(*geometry_);
        connectivity_ = std::make_unique<MeshConnectivity>(*geometry_);

        // Wire up operations to use connectivity for validation
        meshMutator_->setConnectivity(connectivity_.get());

        // Add some test nodes
        node1Id_ = meshMutator_->addNode(Point3D(0.0, 0.0, 0.0));
        node2Id_ = meshMutator_->addNode(Point3D(1.0, 0.0, 0.0));
        node3Id_ = meshMutator_->addNode(Point3D(0.0, 1.0, 0.0));
        node4Id_ = meshMutator_->addNode(Point3D(0.0, 0.0, 1.0));

        // Rebuild connectivity after adding nodes
        connectivity_->rebuildConnectivity();
    }

    void TearDown() override
    {
        meshMutator_.reset();
        connectivity_.reset();
        geometry_.reset();
    }

    std::unique_ptr<MeshData3D> geometry_;
    std::unique_ptr<MeshMutator3D> meshMutator_;
    std::unique_ptr<MeshConnectivity> connectivity_;
    size_t node1Id_, node2Id_, node3Id_, node4Id_;
};

TEST_F(ScopedTransactionTest, AutoRollbackOnScopeExitTest)
{
    EXPECT_EQ(geometry_->getElementCount(), 0);

    {
        ScopedTransaction scopedTx(meshMutator_.get());

        // Add an element within scoped transaction
        auto element = std::make_unique<MockTetrahedralElement>(
            std::array<size_t, 4>{node1Id_, node2Id_, node3Id_, node4Id_});
        size_t elementId = meshMutator_->addElement(std::move(element));

        EXPECT_EQ(geometry_->getElementCount(), 1);

        // No explicit commit - should auto-rollback when scope ends
    }

    // Element should be removed after auto-rollback
    EXPECT_EQ(geometry_->getElementCount(), 0);
}

TEST_F(ScopedTransactionTest, ExplicitCommitTest)
{
    EXPECT_EQ(geometry_->getElementCount(), 0);

    {
        ScopedTransaction scopedTx(meshMutator_.get());

        // Add an element within scoped transaction
        auto element = std::make_unique<MockTetrahedralElement>(
            std::array<size_t, 4>{node1Id_, node2Id_, node3Id_, node4Id_});
        size_t elementId = meshMutator_->addElement(std::move(element));

        EXPECT_EQ(geometry_->getElementCount(), 1);

        // Explicitly commit
        scopedTx.commit();

        // Transaction should be committed, no rollback on scope exit
    }

    // Element should remain after explicit commit
    EXPECT_EQ(geometry_->getElementCount(), 1);
}

TEST_F(ScopedTransactionTest, MultipleOperationsWithCommitTest)
{
    size_t initialNodeCount = geometry_->getNodeCount();
    size_t initialElementCount = geometry_->getElementCount();

    size_t newNodeId, elementId;

    {
        ScopedTransaction scopedTx(meshMutator_.get());

        // Perform multiple operations
        newNodeId = meshMutator_->addNode(Point3D(2.0, 2.0, 2.0));
        meshMutator_->moveNode(node1Id_, Point3D(1.5, 1.5, 1.5));

        auto element = std::make_unique<MockTetrahedralElement>(
            std::array<size_t, 4>{node1Id_, node2Id_, node3Id_, newNodeId});
        elementId = meshMutator_->addElement(std::move(element));

        EXPECT_EQ(geometry_->getNodeCount(), initialNodeCount + 1);
        EXPECT_EQ(geometry_->getElementCount(), initialElementCount + 1);

        // Commit all operations
        scopedTx.commit();
    }

    // All operations should be preserved after commit
    EXPECT_EQ(geometry_->getNodeCount(), initialNodeCount + 1);
    EXPECT_EQ(geometry_->getElementCount(), initialElementCount + 1);
    EXPECT_NE(geometry_->getNode(newNodeId), nullptr);
    EXPECT_NE(geometry_->getElement(elementId), nullptr);

    // Verify node coordinates were modified
    const Node3D* node1 = geometry_->getNode(node1Id_);
    ASSERT_NE(node1, nullptr);
    Point3D expectedCoords(1.5, 1.5, 1.5);
    EXPECT_TRUE(node1->getCoordinates().isApprox(expectedCoords));
}

TEST_F(ScopedTransactionTest, PartialWorkRollbackTest)
{
    size_t initialNodeCount = geometry_->getNodeCount();
    size_t initialElementCount = geometry_->getElementCount();

    {
        ScopedTransaction scopedTx(meshMutator_.get());

        // Add some nodes and elements
        size_t newNode1 = meshMutator_->addNode(Point3D(2.0, 0.0, 0.0));
        size_t newNode2 = meshMutator_->addNode(Point3D(0.0, 2.0, 0.0));

        auto element1 = std::make_unique<MockTetrahedralElement>(
            std::array<size_t, 4>{node1Id_, node2Id_, node3Id_, newNode1});
        size_t elementId1 = meshMutator_->addElement(std::move(element1));

        auto element2 = std::make_unique<MockTetrahedralElement>(
            std::array<size_t, 4>{node1Id_, node3Id_, node4Id_, newNode2});
        size_t elementId2 = meshMutator_->addElement(std::move(element2));

        // Verify operations were applied
        EXPECT_EQ(geometry_->getNodeCount(), initialNodeCount + 2);
        EXPECT_EQ(geometry_->getElementCount(), initialElementCount + 2);

        // Don't commit - let it auto-rollback
    }

    // All operations should be rolled back
    EXPECT_EQ(geometry_->getNodeCount(), initialNodeCount);
    EXPECT_EQ(geometry_->getElementCount(), initialElementCount);
}

} // namespace Operations
