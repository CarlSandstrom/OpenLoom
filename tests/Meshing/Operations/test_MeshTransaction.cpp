#include "Meshing/Data/MeshConnectivity.h"
#include "Meshing/Data/MeshGeometry.h"
#include "Meshing/Data/MeshOperations.h"
#include "Meshing/Data/TetrahedralElement.h"
#include "Meshing/Operations/MeshTransaction.h"
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
        geometry_ = std::make_unique<MeshGeometry>();
        operations_ = std::make_unique<MeshOperations>(*geometry_);
        connectivity_ = std::make_unique<MeshConnectivity>(*geometry_);

        // Wire up operations to use connectivity for validation
        operations_->setConnectivity(connectivity_.get());

        // Add some test nodes
        node1Id_ = operations_->addNode({0.0, 0.0, 0.0});
        node2Id_ = operations_->addNode({1.0, 0.0, 0.0});
        node3Id_ = operations_->addNode({0.0, 1.0, 0.0});
        node4Id_ = operations_->addNode({0.0, 0.0, 1.0});
        node5Id_ = operations_->addNode({1.0, 1.0, 1.0});

        // Rebuild connectivity after adding nodes
        connectivity_->rebuildConnectivity();
    }

    void TearDown() override
    {
        operations_.reset();
        connectivity_.reset();
        geometry_.reset();
    }

    std::unique_ptr<MeshGeometry> geometry_;
    std::unique_ptr<MeshOperations> operations_;
    std::unique_ptr<MeshConnectivity> connectivity_;
    size_t node1Id_, node2Id_, node3Id_, node4Id_, node5Id_;
};

TEST_F(MeshTransactionTest, ConstructorTest)
{
    MeshTransaction transaction(operations_.get());

    EXPECT_FALSE(transaction.isActive());
}

TEST_F(MeshTransactionTest, BeginTransactionTest)
{
    MeshTransaction transaction(operations_.get());

    EXPECT_FALSE(transaction.isActive());

    transaction.begin();

    EXPECT_TRUE(transaction.isActive());
}

TEST_F(MeshTransactionTest, CommitTransactionTest)
{
    MeshTransaction transaction(operations_.get());
    transaction.begin();

    auto element = std::make_unique<MockTetrahedralElement>(
        std::array<size_t, 4>{node1Id_, node2Id_, node3Id_, node4Id_});

    EXPECT_EQ(geometry_->getElementCount(), 0);
    size_t elementId = operations_->addElement(std::move(element));

    EXPECT_EQ(geometry_->getElementCount(), 1);
    EXPECT_TRUE(transaction.isActive());

    transaction.commit();

    EXPECT_EQ(geometry_->getElementCount(), 1);
    EXPECT_FALSE(transaction.isActive());
}

TEST_F(MeshTransactionTest, RollbackTransactionTest)
{
    MeshTransaction transaction(operations_.get());
    transaction.begin();

    auto element = std::make_unique<MockTetrahedralElement>(
        std::array<size_t, 4>{node1Id_, node2Id_, node3Id_, node4Id_});

    EXPECT_EQ(geometry_->getElementCount(), 0);
    size_t elementId = operations_->addElement(std::move(element));

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
        MeshTransaction transaction(operations_.get());
        transaction.begin();

        // Add an element during transaction
        auto element = std::make_unique<MockTetrahedralElement>(
            std::array<size_t, 4>{node1Id_, node2Id_, node3Id_, node4Id_});
        size_t elementId = operations_->addElement(std::move(element));

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

    MeshTransaction transaction(operations_.get());
    transaction.begin();

    // Add a node during transaction
    size_t newNodeId = operations_->addNode({2.0, 2.0, 2.0});

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
    const Node* node = geometry_->getNode(node1Id_);
    ASSERT_NE(node, nullptr);
    auto originalCoords = node->getCoordinates();

    MeshTransaction transaction(operations_.get());
    transaction.begin();

    // Modify an existing node during transaction
    std::array<double, 3> newCoords = {1.5, 1.5, 1.5};
    operations_->moveNode(node1Id_, newCoords);

    // Verify the node was modified
    node = geometry_->getNode(node1Id_);
    EXPECT_EQ(node->getCoordinates(), newCoords);
    EXPECT_TRUE(transaction.isActive());

    transaction.rollback();

    EXPECT_FALSE(transaction.isActive());
    // Node coordinates should be restored after rollback
    node = geometry_->getNode(node1Id_);
    EXPECT_EQ(node->getCoordinates(), originalCoords);
}

TEST_F(MeshTransactionTest, ElementRemovalAndRollbackTest)
{
    // First add an element outside transaction
    auto element = std::make_unique<MockTetrahedralElement>(
        std::array<size_t, 4>{node1Id_, node2Id_, node3Id_, node4Id_});
    size_t elementId = operations_->addElement(std::move(element));

    EXPECT_EQ(geometry_->getElementCount(), 1);
    EXPECT_NE(geometry_->getElement(elementId), nullptr);

    // Now remove it within a transaction
    MeshTransaction transaction(operations_.get());
    transaction.begin();

    operations_->removeElement(elementId);

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
    const Node* node2 = geometry_->getNode(node2Id_);
    auto originalNode2Coords = node2->getCoordinates();

    MeshTransaction transaction(operations_.get());
    transaction.begin();

    // Perform multiple operations
    size_t newNodeId = operations_->addNode({3.0, 3.0, 3.0});
    operations_->moveNode(node2Id_, {2.5, 2.5, 2.5});

    auto element = std::make_unique<MockTetrahedralElement>(
        std::array<size_t, 4>{node1Id_, node3Id_, node4Id_, newNodeId});
    size_t elementId = operations_->addElement(std::move(element));

    // Verify operations were applied
    EXPECT_EQ(geometry_->getNodeCount(), initialNodeCount + 1);
    EXPECT_EQ(geometry_->getElementCount(), initialElementCount + 1);
    node2 = geometry_->getNode(node2Id_);
    EXPECT_NE(node2->getCoordinates(), originalNode2Coords);
    EXPECT_TRUE(transaction.isActive());

    transaction.rollback();

    EXPECT_FALSE(transaction.isActive());
    // All operations should be rolled back
    EXPECT_EQ(geometry_->getNodeCount(), initialNodeCount);
    EXPECT_EQ(geometry_->getElementCount(), initialElementCount);
    node2 = geometry_->getNode(node2Id_);
    EXPECT_EQ(node2->getCoordinates(), originalNode2Coords);
    EXPECT_EQ(geometry_->getNode(newNodeId), nullptr);
    EXPECT_EQ(geometry_->getElement(elementId), nullptr);
}

TEST_F(MeshTransactionTest, NestedTransactionBehaviorTest)
{
    EXPECT_EQ(geometry_->getElementCount(), 0);

    MeshTransaction transaction1(operations_.get());
    transaction1.begin();

    // Add element in first transaction
    auto element1 = std::make_unique<MockTetrahedralElement>(
        std::array<size_t, 4>{node1Id_, node2Id_, node3Id_, node4Id_});
    size_t elementId1 = operations_->addElement(std::move(element1));

    EXPECT_EQ(geometry_->getElementCount(), 1);
    transaction1.commit();
    EXPECT_EQ(geometry_->getElementCount(), 1);

    // Second transaction
    MeshTransaction transaction2(operations_.get());
    transaction2.begin();

    // Add another element
    auto element2 = std::make_unique<MockTetrahedralElement>(
        std::array<size_t, 4>{node2Id_, node3Id_, node4Id_, node5Id_});
    size_t elementId2 = operations_->addElement(std::move(element2));

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
        geometry_ = std::make_unique<MeshGeometry>();
        operations_ = std::make_unique<MeshOperations>(*geometry_);
        connectivity_ = std::make_unique<MeshConnectivity>(*geometry_);

        // Wire up operations to use connectivity for validation
        operations_->setConnectivity(connectivity_.get());

        // Add some test nodes
        node1Id_ = operations_->addNode({0.0, 0.0, 0.0});
        node2Id_ = operations_->addNode({1.0, 0.0, 0.0});
        node3Id_ = operations_->addNode({0.0, 1.0, 0.0});
        node4Id_ = operations_->addNode({0.0, 0.0, 1.0});

        // Rebuild connectivity after adding nodes
        connectivity_->rebuildConnectivity();
    }

    void TearDown() override
    {
        operations_.reset();
        connectivity_.reset();
        geometry_.reset();
    }

    std::unique_ptr<MeshGeometry> geometry_;
    std::unique_ptr<MeshOperations> operations_;
    std::unique_ptr<MeshConnectivity> connectivity_;
    size_t node1Id_, node2Id_, node3Id_, node4Id_;
};

TEST_F(ScopedTransactionTest, AutoRollbackOnScopeExitTest)
{
    EXPECT_EQ(geometry_->getElementCount(), 0);

    {
        ScopedTransaction scopedTx(operations_.get());

        // Add an element within scoped transaction
        auto element = std::make_unique<MockTetrahedralElement>(
            std::array<size_t, 4>{node1Id_, node2Id_, node3Id_, node4Id_});
        size_t elementId = operations_->addElement(std::move(element));

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
        ScopedTransaction scopedTx(operations_.get());

        // Add an element within scoped transaction
        auto element = std::make_unique<MockTetrahedralElement>(
            std::array<size_t, 4>{node1Id_, node2Id_, node3Id_, node4Id_});
        size_t elementId = operations_->addElement(std::move(element));

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
        ScopedTransaction scopedTx(operations_.get());

        // Perform multiple operations
        newNodeId = operations_->addNode({2.0, 2.0, 2.0});
        operations_->moveNode(node1Id_, {1.5, 1.5, 1.5});

        auto element = std::make_unique<MockTetrahedralElement>(
            std::array<size_t, 4>{node1Id_, node2Id_, node3Id_, newNodeId});
        elementId = operations_->addElement(std::move(element));

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
    const Node* node1 = geometry_->getNode(node1Id_);
    std::array<double, 3> expectedCoords = {1.5, 1.5, 1.5};
    EXPECT_EQ(node1->getCoordinates(), expectedCoords);
}

TEST_F(ScopedTransactionTest, PartialWorkRollbackTest)
{
    size_t initialNodeCount = geometry_->getNodeCount();
    size_t initialElementCount = geometry_->getElementCount();

    {
        ScopedTransaction scopedTx(operations_.get());

        // Add some nodes and elements
        size_t newNode1 = operations_->addNode({2.0, 0.0, 0.0});
        size_t newNode2 = operations_->addNode({0.0, 2.0, 0.0});

        auto element1 = std::make_unique<MockTetrahedralElement>(
            std::array<size_t, 4>{node1Id_, node2Id_, node3Id_, newNode1});
        size_t elementId1 = operations_->addElement(std::move(element1));

        auto element2 = std::make_unique<MockTetrahedralElement>(
            std::array<size_t, 4>{node1Id_, node3Id_, node4Id_, newNode2});
        size_t elementId2 = operations_->addElement(std::move(element2));

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
