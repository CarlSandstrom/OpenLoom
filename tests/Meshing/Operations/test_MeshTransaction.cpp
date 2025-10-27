#include "Meshing/Data/MeshData.h"
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

    std::unique_ptr<Element> clone() const override
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
        meshData_ = std::make_unique<MeshData>();

        // Add some test nodes
        node1Id_ = meshData_->addNode({0.0, 0.0, 0.0});
        node2Id_ = meshData_->addNode({1.0, 0.0, 0.0});
        node3Id_ = meshData_->addNode({0.0, 1.0, 0.0});
        node4Id_ = meshData_->addNode({0.0, 0.0, 1.0});
        node5Id_ = meshData_->addNode({1.0, 1.0, 1.0});
    }

    void TearDown() override
    {
        meshData_.reset();
    }

    std::unique_ptr<MeshData> meshData_;
    size_t node1Id_, node2Id_, node3Id_, node4Id_, node5Id_;
};

TEST_F(MeshTransactionTest, ConstructorTest)
{
    MeshTransaction transaction(meshData_.get());

    EXPECT_FALSE(transaction.isActive());
}

TEST_F(MeshTransactionTest, BeginTransactionTest)
{
    MeshTransaction transaction(meshData_.get());

    EXPECT_FALSE(transaction.isActive());

    transaction.begin();

    EXPECT_TRUE(transaction.isActive());
}

TEST_F(MeshTransactionTest, CommitTransactionTest)
{
    MeshTransaction transaction(meshData_.get());
    transaction.begin();

    // Add an element during transaction
    auto element = std::make_unique<MockTetrahedralElement>(
        std::array<size_t, 4>{node1Id_, node2Id_, node3Id_, node4Id_});
    size_t elementId = meshData_->addElement(std::move(element));

    EXPECT_TRUE(transaction.isActive());

    transaction.commit();

    EXPECT_FALSE(transaction.isActive());

    // Element should still exist after commit
    // Note: Would need additional MeshData methods to verify element existence
}

TEST_F(MeshTransactionTest, RollbackTransactionTest)
{
    MeshTransaction transaction(meshData_.get());
    transaction.begin();

    // Add an element during transaction
    auto element = std::make_unique<MockTetrahedralElement>(
        std::array<size_t, 4>{node1Id_, node2Id_, node3Id_, node4Id_});
    size_t elementId = meshData_->addElement(std::move(element));

    EXPECT_TRUE(transaction.isActive());

    transaction.rollback();

    EXPECT_FALSE(transaction.isActive());

    // Element should be removed after rollback
    // Note: Would need additional MeshData methods to verify element removal
}

TEST_F(MeshTransactionTest, AutoRollbackOnDestructorTest)
{
    {
        MeshTransaction transaction(meshData_.get());
        transaction.begin();

        // Add an element during transaction
        auto element = std::make_unique<MockTetrahedralElement>(
            std::array<size_t, 4>{node1Id_, node2Id_, node3Id_, node4Id_});
        size_t elementId = meshData_->addElement(std::move(element));

        EXPECT_TRUE(transaction.isActive());

        // Transaction should auto-rollback when going out of scope
    }

    // Element should be removed after auto-rollback
    // Note: Would need additional MeshData methods to verify element removal
}

TEST_F(MeshTransactionTest, NodeAdditionAndRollbackTest)
{
    MeshTransaction transaction(meshData_.get());
    transaction.begin();

    // Add a node during transaction
    size_t newNodeId = meshData_->addNode({2.0, 2.0, 2.0});

    EXPECT_TRUE(transaction.isActive());

    transaction.rollback();

    EXPECT_FALSE(transaction.isActive());

    // Node should be removed after rollback
    // Note: Would need additional MeshData methods to verify node removal
}

TEST_F(MeshTransactionTest, NodeModificationAndRollbackTest)
{
    MeshTransaction transaction(meshData_.get());
    transaction.begin();

    // Modify an existing node during transaction
    meshData_->moveNode(node1Id_, {1.5, 1.5, 1.5});

    EXPECT_TRUE(transaction.isActive());

    transaction.rollback();

    EXPECT_FALSE(transaction.isActive());

    // Node coordinates should be restored after rollback
    // Note: Would need additional MeshData methods to verify coordinate restoration
}

TEST_F(MeshTransactionTest, ElementRemovalAndRollbackTest)
{
    // First add an element outside transaction
    auto element = std::make_unique<MockTetrahedralElement>(
        std::array<size_t, 4>{node1Id_, node2Id_, node3Id_, node4Id_});
    size_t elementId = meshData_->addElement(std::move(element));

    // Now remove it within a transaction
    MeshTransaction transaction(meshData_.get());
    transaction.begin();

    meshData_->removeElement(elementId);

    EXPECT_TRUE(transaction.isActive());

    transaction.rollback();

    EXPECT_FALSE(transaction.isActive());

    // Element should be restored after rollback
    // Note: Would need additional MeshData methods to verify element restoration
}

TEST_F(MeshTransactionTest, MultipleOperationsRollbackTest)
{
    MeshTransaction transaction(meshData_.get());
    transaction.begin();

    // Perform multiple operations
    size_t newNodeId = meshData_->addNode({3.0, 3.0, 3.0});
    meshData_->moveNode(node2Id_, {2.5, 2.5, 2.5});

    auto element = std::make_unique<MockTetrahedralElement>(
        std::array<size_t, 4>{node1Id_, node3Id_, node4Id_, newNodeId});
    size_t elementId = meshData_->addElement(std::move(element));

    EXPECT_TRUE(transaction.isActive());

    transaction.rollback();

    EXPECT_FALSE(transaction.isActive());

    // All operations should be rolled back
    // Note: Would need additional MeshData methods to verify complete rollback
}

TEST_F(MeshTransactionTest, NestedTransactionBehaviorTest)
{
    MeshTransaction transaction1(meshData_.get());
    transaction1.begin();

    // Add element in first transaction
    auto element1 = std::make_unique<MockTetrahedralElement>(
        std::array<size_t, 4>{node1Id_, node2Id_, node3Id_, node4Id_});
    size_t elementId1 = meshData_->addElement(std::move(element1));

    transaction1.commit();

    // Second transaction
    MeshTransaction transaction2(meshData_.get());
    transaction2.begin();

    // Add another element
    auto element2 = std::make_unique<MockTetrahedralElement>(
        std::array<size_t, 4>{node2Id_, node3Id_, node4Id_, node5Id_});
    size_t elementId2 = meshData_->addElement(std::move(element2));

    transaction2.rollback();

    // First element should remain, second should be rolled back
    // Note: Would need additional MeshData methods to verify state
}

// Tests for ScopedTransaction
class ScopedTransactionTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        meshData_ = std::make_unique<MeshData>();

        // Add some test nodes
        node1Id_ = meshData_->addNode({0.0, 0.0, 0.0});
        node2Id_ = meshData_->addNode({1.0, 0.0, 0.0});
        node3Id_ = meshData_->addNode({0.0, 1.0, 0.0});
        node4Id_ = meshData_->addNode({0.0, 0.0, 1.0});
    }

    void TearDown() override
    {
        meshData_.reset();
    }

    std::unique_ptr<MeshData> meshData_;
    size_t node1Id_, node2Id_, node3Id_, node4Id_;
};

TEST_F(ScopedTransactionTest, AutoRollbackOnScopeExitTest)
{
    {
        ScopedTransaction scopedTx(meshData_.get());

        // Add an element within scoped transaction
        auto element = std::make_unique<MockTetrahedralElement>(
            std::array<size_t, 4>{node1Id_, node2Id_, node3Id_, node4Id_});
        size_t elementId = meshData_->addElement(std::move(element));

        // Transaction should be active
        // Note: ScopedTransaction doesn't expose isActive(), but underlying transaction should be active

        // No explicit commit - should auto-rollback when scope ends
    }

    // Element should be removed after auto-rollback
    // Note: Would need additional MeshData methods to verify element removal
}

TEST_F(ScopedTransactionTest, ExplicitCommitTest)
{
    {
        ScopedTransaction scopedTx(meshData_.get());

        // Add an element within scoped transaction
        auto element = std::make_unique<MockTetrahedralElement>(
            std::array<size_t, 4>{node1Id_, node2Id_, node3Id_, node4Id_});
        size_t elementId = meshData_->addElement(std::move(element));

        // Explicitly commit
        scopedTx.commit();

        // Transaction should be committed, no rollback on scope exit
    }

    // Element should remain after explicit commit
    // Note: Would need additional MeshData methods to verify element persistence
}

TEST_F(ScopedTransactionTest, MultipleOperationsWithCommitTest)
{
    {
        ScopedTransaction scopedTx(meshData_.get());

        // Perform multiple operations
        size_t newNodeId = meshData_->addNode({2.0, 2.0, 2.0});
        meshData_->moveNode(node1Id_, {1.5, 1.5, 1.5});

        auto element = std::make_unique<MockTetrahedralElement>(
            std::array<size_t, 4>{node1Id_, node2Id_, node3Id_, newNodeId});
        size_t elementId = meshData_->addElement(std::move(element));

        // Commit all operations
        scopedTx.commit();
    }

    // All operations should be preserved after commit
    // Note: Would need additional MeshData methods to verify state
}

TEST_F(ScopedTransactionTest, PartialWorkRollbackTest)
{
    {
        ScopedTransaction scopedTx(meshData_.get());

        // Add some nodes and elements
        size_t newNode1 = meshData_->addNode({2.0, 0.0, 0.0});
        size_t newNode2 = meshData_->addNode({0.0, 2.0, 0.0});

        auto element1 = std::make_unique<MockTetrahedralElement>(
            std::array<size_t, 4>{node1Id_, node2Id_, node3Id_, newNode1});
        size_t elementId1 = meshData_->addElement(std::move(element1));

        auto element2 = std::make_unique<MockTetrahedralElement>(
            std::array<size_t, 4>{node1Id_, node3Id_, node4Id_, newNode2});
        size_t elementId2 = meshData_->addElement(std::move(element2));

        // Don't commit - let it auto-rollback
    }

    // All operations should be rolled back
    // Note: Would need additional MeshData methods to verify complete rollback
}

} // namespace Operations
