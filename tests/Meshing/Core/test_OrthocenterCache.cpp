#include <gtest/gtest.h>

#include "Common/Types.h"
#include "Meshing/Core/3D/General/ElementGeometry3D.h"
#include "Meshing/Core/3D/General/OrthocenterCache.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Meshing/Data/3D/TetrahedronElement.h"

#include <array>
#include <memory>

using namespace Meshing;

class OrthocenterCacheTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        mutator_ = std::make_unique<MeshMutator3D>(meshData_);
        nodeIds_ = {mutator_->addNode(Point3D(0.0, 0.0, 0.0)), mutator_->addNode(Point3D(2.0, 0.0, 0.0)),
                    mutator_->addNode(Point3D(0.0, 2.0, 0.0)), mutator_->addNode(Point3D(0.0, 0.0, 2.0))};
        elementId_ = mutator_->addElement(std::make_unique<TetrahedronElement>(nodeIds_));
    }

    const TetrahedronElement& tetrahedron() const
    {
        return dynamic_cast<const TetrahedronElement&>(*meshData_.getElement(elementId_));
    }

    Point3D uncachedOrthocenter() const { return *ElementGeometry3D(meshData_).computeOrthocenter(tetrahedron()); }

    MeshData3D meshData_;
    std::unique_ptr<MeshMutator3D> mutator_;
    std::array<NodeId, 4> nodeIds_;
    ElementId elementId_{0};
};

TEST_F(OrthocenterCacheTest, ReturnsTheComputedOrthocenterAgain)
{
    OrthocenterCache cache;

    auto first = cache.orthocenterOf(elementId_, tetrahedron(), meshData_);
    auto second = cache.orthocenterOf(elementId_, tetrahedron(), meshData_);

    ASSERT_TRUE(first.has_value());
    EXPECT_EQ(*first, uncachedOrthocenter());
    EXPECT_EQ(*second, *first);
}

// Smoothing moves vertices while tetrahedra keep their ids.
TEST_F(OrthocenterCacheTest, RecomputesAfterAVertexMoves)
{
    OrthocenterCache cache;
    auto before = *cache.orthocenterOf(elementId_, tetrahedron(), meshData_);

    mutator_->moveNode(nodeIds_[3], Point3D(0.0, 0.0, 4.0));
    auto after = cache.orthocenterOf(elementId_, tetrahedron(), meshData_);

    ASSERT_TRUE(after.has_value());
    EXPECT_NE(*after, before);
    EXPECT_EQ(*after, uncachedOrthocenter());
}

// A weight changes the power distance, and with it the orthocenter.
TEST_F(OrthocenterCacheTest, RecomputesAfterAVertexWeightChanges)
{
    OrthocenterCache cache;
    auto before = *cache.orthocenterOf(elementId_, tetrahedron(), meshData_);

    mutator_->setNodeWeight(nodeIds_[0], 0.5);
    auto after = cache.orthocenterOf(elementId_, tetrahedron(), meshData_);

    ASSERT_TRUE(after.has_value());
    EXPECT_NE(*after, before);
    EXPECT_EQ(*after, uncachedOrthocenter());
}
