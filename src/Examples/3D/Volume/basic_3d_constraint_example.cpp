/**
 * @file basic_3d_constraint_example.cpp
 * @brief Basic example demonstrating 3D constraint checking and mesh operations
 *
 * This example demonstrates:
 * 1. Creating a simple tetrahedral mesh
 * 2. Testing subsegment encroachment (diametral sphere)
 * 3. Testing subfacet encroachment (equatorial sphere)
 * 4. Using Bowyer-Watson vertex insertion
 */

#include "Common/Types.h"
#include "Meshing/Core/3D/General/ConstraintChecker3D.h"
#include "Meshing/Core/3D/General/GeometryStructures3D.h"
#include "Meshing/Core/3D/General/MeshOperations3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Meshing/Data/3D/Node3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"
#include <iostream>
#include <memory>

using namespace Meshing;

void printSeparator(const std::string& title)
{
    std::cout << "\n========================================\n";
    std::cout << title << "\n";
    std::cout << "========================================\n";
}

int main()
{
    printSeparator("Basic 3D Constraint Checking Example");

    // Create a simple tetrahedral mesh
    MeshData3D meshData;
    MeshMutator3D mutator(meshData);

    // Add four nodes forming a tetrahedron
    Point3D p0(0.0, 0.0, 0.0);
    Point3D p1(1.0, 0.0, 0.0);
    Point3D p2(0.5, 1.0, 0.0);
    Point3D p3(0.5, 0.5, 1.0);

    size_t n0 = mutator.addNode(p0);
    size_t n1 = mutator.addNode(p1);
    size_t n2 = mutator.addNode(p2);
    size_t n3 = mutator.addNode(p3);

    std::cout << "Created 4 nodes:\n";
    std::cout << "  Node " << n0 << ": " << p0.transpose() << "\n";
    std::cout << "  Node " << n1 << ": " << p1.transpose() << "\n";
    std::cout << "  Node " << n2 << ": " << p2.transpose() << "\n";
    std::cout << "  Node " << n3 << ": " << p3.transpose() << "\n";

    // Add a tetrahedron
    auto tet = std::make_unique<TetrahedralElement>(std::array<size_t, 4>{n0, n1, n2, n3});
    size_t tetId = mutator.addElement(std::move(tet));
    std::cout << "\nCreated tetrahedron with ID: " << tetId << "\n";

    // ==================================================
    // Test 1: Subsegment Encroachment
    // ==================================================
    printSeparator("Test 1: Subsegment Encroachment");

    ConstraintChecker3D checker(meshData);

    // Create a constrained subsegment between nodes 0 and 1
    ConstrainedSubsegment3D subsegment;
    subsegment.nodeId1 = n0;
    subsegment.nodeId2 = n1;
    subsegment.geometryId = "edge_01";

    std::cout << "Testing subsegment encroachment for edge (" << n0 << " -> " << n1 << ")\n";

    // Test point at midpoint (should encroach)
    Point3D midpoint = (p0 + p1) * 0.5;
    bool encroached1 = checker.isSubsegmentEncroached(subsegment, midpoint);
    std::cout << "  Point at midpoint " << midpoint.transpose()
              << ": " << (encroached1 ? "ENCROACHES" : "does not encroach") << "\n";

    // Test point far from segment (should not encroach)
    Point3D farPoint(10.0, 10.0, 10.0);
    bool encroached2 = checker.isSubsegmentEncroached(subsegment, farPoint);
    std::cout << "  Point far away " << farPoint.transpose()
              << ": " << (encroached2 ? "ENCROACHES" : "does not encroach") << "\n";

    // Test point near but outside diametral sphere
    Point3D nearPoint = (p0 + p1) * 0.5 + Point3D(0.0, 0.6, 0.0);
    bool encroached3 = checker.isSubsegmentEncroached(subsegment, nearPoint);
    std::cout << "  Point near edge " << nearPoint.transpose()
              << ": " << (encroached3 ? "ENCROACHES" : "does not encroach") << "\n";

    // ==================================================
    // Test 2: Subfacet Encroachment
    // ==================================================
    printSeparator("Test 2: Subfacet Encroachment");

    // Create a constrained subfacet using nodes 0, 1, 2
    ConstrainedSubfacet3D subfacet;
    subfacet.nodeId1 = n0;
    subfacet.nodeId2 = n1;
    subfacet.nodeId3 = n2;
    subfacet.geometryId = "face_012";

    std::cout << "Testing subfacet encroachment for triangle ("
              << n0 << ", " << n1 << ", " << n2 << ")\n";

    // Test point above the triangle (non-coplanar, might encroach)
    Point3D abovePoint = (p0 + p1 + p2) / 3.0 + Point3D(0.0, 0.0, 0.1);
    bool encroachedFace1 = checker.isSubfacetEncroached(subfacet, abovePoint);
    std::cout << "  Point slightly above " << abovePoint.transpose()
              << ": " << (encroachedFace1 ? "ENCROACHES" : "does not encroach") << "\n";

    // Test point in the plane (coplanar, should not encroach)
    Point3D coplanarPoint = (p0 + p1 + p2) / 3.0;
    bool encroachedFace2 = checker.isSubfacetEncroached(subfacet, coplanarPoint);
    std::cout << "  Point in plane " << coplanarPoint.transpose()
              << ": " << (encroachedFace2 ? "ENCROACHES" : "does not encroach") << "\n";

    // Test point far from triangle
    bool encroachedFace3 = checker.isSubfacetEncroached(subfacet, farPoint);
    std::cout << "  Point far away " << farPoint.transpose()
              << ": " << (encroachedFace3 ? "ENCROACHES" : "does not encroach") << "\n";

    // ==================================================
    // Test 3: Bowyer-Watson Vertex Insertion
    // ==================================================
    printSeparator("Test 3: Bowyer-Watson Vertex Insertion");

    MeshOperations3D operations(meshData);

    std::cout << "Initial mesh has " << meshData.getNodeCount() << " nodes and "
              << meshData.getElementCount() << " tetrahedra\n";

    // Insert a new vertex at the centroid
    Point3D centroid = (p0 + p1 + p2 + p3) / 4.0;
    std::cout << "\nInserting vertex at centroid: " << centroid.transpose() << "\n";

    size_t newNodeId = operations.insertVertexBowyerWatson(centroid);
    std::cout << "Inserted vertex with ID: " << newNodeId << "\n";

    std::cout << "\nAfter insertion, mesh has " << meshData.getNodeCount() << " nodes and "
              << meshData.getElementCount() << " tetrahedra\n";

    // Verify the new node was added
    const auto* newNode = meshData.getNode(newNodeId);
    if (newNode)
    {
        std::cout << "New node coordinates: " << newNode->getCoordinates().transpose() << "\n";
    }

    printSeparator("Example Complete");
    std::cout << "\nSummary:\n";
    std::cout << "  - Created a tetrahedral mesh\n";
    std::cout << "  - Tested subsegment encroachment (diametral sphere)\n";
    std::cout << "  - Tested subfacet encroachment (equatorial sphere)\n";
    std::cout << "  - Demonstrated Bowyer-Watson vertex insertion\n";

    return 0;
}
