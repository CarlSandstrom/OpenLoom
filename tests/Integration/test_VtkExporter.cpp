#include <array>
#include <fstream>
#include <gtest/gtest.h>
#include <string>

#include "Common/Types.h"
#include "Export/VtkExporter.h"
#include "Meshing/Data/CurveSegmentManager.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/MeshMutator2D.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"

TEST(VtkExporter, WritesBasicVtu)
{
    // Create mesh data directly without relying on mesher classes
    Meshing::MeshData3D meshData;
    Meshing::MeshMutator3D mutator(meshData);

    // Add 4 nodes forming a tetrahedron
    size_t n0 = mutator.addNode(Meshing::Point3D(0.0, 0.0, 0.0));
    size_t n1 = mutator.addNode(Meshing::Point3D(1.0, 0.0, 0.0));
    size_t n2 = mutator.addNode(Meshing::Point3D(0.0, 1.0, 0.0));
    size_t n3 = mutator.addNode(Meshing::Point3D(0.0, 0.0, 1.0));

    // Add a tetrahedral element
    mutator.addElement(std::make_unique<Meshing::TetrahedralElement>(std::array<size_t, 4>{n0, n1, n2, n3}));

    Export::VtkExporter exporter;
    const std::string filePath = "test_output.vtu";
    ASSERT_TRUE(exporter.exportMesh(meshData, filePath));

    // Basic file existence and content checks
    std::ifstream in(filePath);
    ASSERT_TRUE(in.is_open());
    std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    EXPECT_TRUE(content.find("<VTKFile") != std::string::npos);
    EXPECT_TRUE(content.find("<UnstructuredGrid>") != std::string::npos);
    EXPECT_TRUE(content.find("<Points>") != std::string::npos);
    EXPECT_TRUE(content.find("<Cells>") != std::string::npos);
    EXPECT_TRUE(content.find("types") != std::string::npos);
}

// Build a 2D mesh with boundary and interior constraints:
//
//   3 --- 2 --- 5
//   |   / |   / |
//   |  /  |  /  |
//   | /   | /   |
//   0 --- 1 --- 4
//
// Outer square (0-1-2-3) = BOUNDARY, interior edge (1-2) = INTERIOR
// This creates two domains: left (0,1,2,3) and right (1,2,4,5)
class VtkExporter2DDomainTest : public ::testing::Test
{
protected:
    Meshing::MeshData2D meshData_;
    std::unique_ptr<Meshing::MeshMutator2D> mutator_;

    void SetUp() override
    {
        mutator_ = std::make_unique<Meshing::MeshMutator2D>(meshData_);

        // Nodes for a rectangle divided by an interior edge
        size_t n0 = mutator_->addNode(Meshing::Point2D(0.0, 0.0));
        size_t n1 = mutator_->addNode(Meshing::Point2D(1.0, 0.0));
        size_t n2 = mutator_->addNode(Meshing::Point2D(1.0, 1.0));
        size_t n3 = mutator_->addNode(Meshing::Point2D(0.0, 1.0));
        size_t n4 = mutator_->addNode(Meshing::Point2D(2.0, 0.0));
        size_t n5 = mutator_->addNode(Meshing::Point2D(2.0, 1.0));

        // Left domain: two triangles
        mutator_->addElement(std::make_unique<Meshing::TriangleElement>(
            std::array<size_t, 3>{n0, n1, n2}));
        mutator_->addElement(std::make_unique<Meshing::TriangleElement>(
            std::array<size_t, 3>{n0, n2, n3}));

        // Right domain: two triangles
        mutator_->addElement(std::make_unique<Meshing::TriangleElement>(
            std::array<size_t, 3>{n1, n4, n5}));
        mutator_->addElement(std::make_unique<Meshing::TriangleElement>(
            std::array<size_t, 3>{n1, n5, n2}));

        // Outer boundary constraints (role defaults to Boundary)
        mutator_->addCurveSegment({n0, n1});
        mutator_->addCurveSegment({n1, n4});
        mutator_->addCurveSegment({n4, n5});
        mutator_->addCurveSegment({n5, n2});
        mutator_->addCurveSegment({n2, n3});
        mutator_->addCurveSegment({n3, n0});

        // Interior constraint dividing the two domains
        mutator_->addCurveSegment({.nodeId1 = n1, .nodeId2 = n2, .role = Meshing::EdgeRole::Interior});
    }
};

TEST_F(VtkExporter2DDomainTest, AutoClassifiesDomainIDs)
{
    Export::VtkExporter exporter;
    const std::string filePath = "test_domain_ids.vtu";
    ASSERT_TRUE(exporter.exportMesh(meshData_, filePath));

    std::ifstream in(filePath);
    ASSERT_TRUE(in.is_open());
    std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());

    // DomainID array should be present
    EXPECT_NE(content.find("DomainID"), std::string::npos);

    // Should have two distinct domain values (0 and 1)
    // The exact assignment order depends on iteration, but both should appear
    auto domainArrayPos = content.find("Name=\"DomainID\"");
    ASSERT_NE(domainArrayPos, std::string::npos);
    auto dataStart = content.find(">", domainArrayPos);
    auto dataEnd = content.find("</DataArray>", domainArrayPos);
    std::string domainData = content.substr(dataStart + 1, dataEnd - dataStart - 1);

    // Both domain 0 and domain 1 should be present
    EXPECT_NE(domainData.find("0"), std::string::npos);
    EXPECT_NE(domainData.find("1"), std::string::npos);
}

TEST(VtkExporter2D, NoDomainIDWithoutConstraints)
{
    Meshing::MeshData2D meshData;
    Meshing::MeshMutator2D mutator(meshData);

    // Simple triangle with no constraints
    size_t n0 = mutator.addNode(Meshing::Point2D(0.0, 0.0));
    size_t n1 = mutator.addNode(Meshing::Point2D(1.0, 0.0));
    size_t n2 = mutator.addNode(Meshing::Point2D(0.0, 1.0));
    mutator.addElement(std::make_unique<Meshing::TriangleElement>(
        std::array<size_t, 3>{n0, n1, n2}));

    Export::VtkExporter exporter;
    const std::string filePath = "test_no_domain_ids.vtu";
    ASSERT_TRUE(exporter.exportMesh(meshData, filePath));

    std::ifstream in(filePath);
    ASSERT_TRUE(in.is_open());
    std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());

    // DomainID should NOT be present when there are no constraints
    EXPECT_EQ(content.find("DomainID"), std::string::npos);
}
