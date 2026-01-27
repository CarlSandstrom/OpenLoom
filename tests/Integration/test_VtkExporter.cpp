#include <array>
#include <fstream>
#include <gtest/gtest.h>
#include <string>

#include "Common/Types.h"
#include "Export/VtkExporter.h"
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
