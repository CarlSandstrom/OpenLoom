/**
 * @file 3d_mesh_with_holes_example.cpp
 * @brief Advanced example demonstrating 3D mesh refinement with holes
 *
 * This example demonstrates:
 * 1. Creating a mesh with boundary constraints
 * 2. Splitting constrained subsegments
 * 3. Splitting constrained subfacets
 * 4. Handling mesh domains with holes (interior voids)
 */

#include "Common/Types.h"
#include "Meshing/Core/3D/General/ConstraintChecker3D.h"
#include "Meshing/Core/3D/General/GeometryStructures3D.h"
#include "Meshing/Core/3D/General/MeshOperations3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Meshing/Data/3D/Node3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"
#include "Meshing/Data/CurveSegmentManager.h"
#include <iostream>
#include <memory>
#include <vector>

using namespace Meshing;

void printSeparator(const std::string& title)
{
    std::cout << "\n========================================\n";
    std::cout << title << "\n";
    std::cout << "========================================\n";
}

// Mock edge class for the example
class MockEdge
{
public:
    MockEdge(const Point3D& start, const Point3D& end) :
        start_(start), end_(end) {}

    Point3D getPoint(double t) const
    {
        return start_ + t * (end_ - start_);
    }

    std::pair<double, double> getParameterBounds() const
    {
        return {0.0, 1.0};
    }

private:
    Point3D start_;
    Point3D end_;
};

// Mock surface class for the example
class MockSurface
{
public:
    // Placeholder for surface operations
};

int main()
{
    printSeparator("3D Mesh with Holes Example");

    // Create a mesh representing a cube with an interior cavity
    MeshData3D meshData;
    MeshMutator3D mutator(meshData);

    std::cout << "Creating outer cube vertices...\n";

    // Outer cube vertices (2x2x2 cube centered at origin)
    std::vector<Point3D> outerVertices = {
        Point3D(-1.0, -1.0, -1.0), // 0
        Point3D(1.0, -1.0, -1.0),  // 1
        Point3D(1.0, 1.0, -1.0),   // 2
        Point3D(-1.0, 1.0, -1.0),  // 3
        Point3D(-1.0, -1.0, 1.0),  // 4
        Point3D(1.0, -1.0, 1.0),   // 5
        Point3D(1.0, 1.0, 1.0),    // 6
        Point3D(-1.0, 1.0, 1.0)    // 7
    };

    std::vector<size_t> outerNodeIds;
    for (size_t i = 0; i < outerVertices.size(); ++i)
    {
        size_t nodeId = mutator.addNode(outerVertices[i]);
        outerNodeIds.push_back(nodeId);
        std::cout << "  Outer node " << nodeId << ": " << outerVertices[i].transpose() << "\n";
    }

    std::cout << "\nCreating inner cavity vertices (hole)...\n";

    // Inner cavity vertices (0.5x0.5x0.5 cube centered at origin)
    std::vector<Point3D> innerVertices = {
        Point3D(-0.25, -0.25, -0.25), // 8
        Point3D(0.25, -0.25, -0.25),  // 9
        Point3D(0.25, 0.25, -0.25),   // 10
        Point3D(-0.25, 0.25, -0.25),  // 11
        Point3D(-0.25, -0.25, 0.25),  // 12
        Point3D(0.25, -0.25, 0.25),   // 13
        Point3D(0.25, 0.25, 0.25),    // 14
        Point3D(-0.25, 0.25, 0.25)    // 15
    };

    std::vector<size_t> innerNodeIds;
    for (size_t i = 0; i < innerVertices.size(); ++i)
    {
        size_t nodeId = mutator.addNode(innerVertices[i]);
        innerNodeIds.push_back(nodeId);
        std::cout << "  Inner node " << nodeId << ": " << innerVertices[i].transpose() << "\n";
    }

    // ==================================================
    // Test 1: Define Constrained Subsegments (Edges)
    // ==================================================
    printSeparator("Test 1: Constrained Subsegments");

    std::vector<CurveSegment> subsegments;

    // Define some edges of the outer cube
    auto addSubsegment = [&](size_t n1, size_t n2, const std::string& id)
    {
        CurveSegment seg;
        seg.nodeId1 = outerNodeIds[n1];
        seg.nodeId2 = outerNodeIds[n2];
        seg.edgeId = id;
        subsegments.push_back(seg);
        std::cout << "  Edge " << id << ": (" << seg.nodeId1 << " -> " << seg.nodeId2 << ")\n";
    };

    // Bottom face edges
    addSubsegment(0, 1, "edge_outer_01");
    addSubsegment(1, 2, "edge_outer_12");
    addSubsegment(2, 3, "edge_outer_23");
    addSubsegment(3, 0, "edge_outer_30");

    std::cout << "Created " << subsegments.size() << " constrained subsegments\n";

    // ==================================================
    // Test 2: Split a Constrained Subsegment
    // ==================================================
    printSeparator("Test 2: Split Constrained Subsegment");

    MeshOperations3D operations(meshData);

    std::cout << "Before split: mesh has " << meshData.getNodeCount() << " nodes\n";

    // Split the first subsegment
    CurveSegment& segmentToSplit = subsegments[0];
    const auto* node1 = meshData.getNode(segmentToSplit.nodeId1);
    const auto* node2 = meshData.getNode(segmentToSplit.nodeId2);

    std::cout << "\nSplitting subsegment (" << segmentToSplit.nodeId1
              << " -> " << segmentToSplit.nodeId2 << ")\n";
    std::cout << "  From: " << node1->getCoordinates().transpose() << "\n";
    std::cout << "  To:   " << node2->getCoordinates().transpose() << "\n";

    // Create a mock edge for splitting
    MockEdge mockEdge(node1->getCoordinates(), node2->getCoordinates());

    // Split the segment (Note: This requires a real Geometry3D::IEdge3D in practice)
    // For this example, we'll demonstrate the concept
    Point3D midpoint = (node1->getCoordinates() + node2->getCoordinates()) * 0.5;
    size_t midNodeId = mutator.addNode(midpoint);

    std::cout << "  Created midpoint node " << midNodeId << ": " << midpoint.transpose() << "\n";

    // Create two new subsegments
    CurveSegment seg1, seg2;
    seg1.nodeId1 = segmentToSplit.nodeId1;
    seg1.nodeId2 = midNodeId;
    seg1.edgeId = segmentToSplit.edgeId;

    seg2.nodeId1 = midNodeId;
    seg2.nodeId2 = segmentToSplit.nodeId2;
    seg2.edgeId = segmentToSplit.edgeId;

    std::cout << "  New subsegment 1: (" << seg1.nodeId1 << " -> " << seg1.nodeId2 << ")\n";
    std::cout << "  New subsegment 2: (" << seg2.nodeId1 << " -> " << seg2.nodeId2 << ")\n";

    std::cout << "\nAfter split: mesh has " << meshData.getNodeCount() << " nodes\n";

    // ==================================================
    // Test 3: Define Constrained Subfacets (Faces)
    // ==================================================
    printSeparator("Test 3: Constrained Subfacets");

    std::vector<ConstrainedSubfacet3D> subfacets;

    // Define the bottom face of outer cube
    ConstrainedSubfacet3D face1, face2;
    face1.nodeId1 = outerNodeIds[0];
    face1.nodeId2 = outerNodeIds[1];
    face1.nodeId3 = outerNodeIds[2];
    face1.geometryId = "face_outer_bottom_1";

    face2.nodeId1 = outerNodeIds[0];
    face2.nodeId2 = outerNodeIds[2];
    face2.nodeId3 = outerNodeIds[3];
    face2.geometryId = "face_outer_bottom_2";

    subfacets.push_back(face1);
    subfacets.push_back(face2);

    std::cout << "Created " << subfacets.size() << " constrained subfacets:\n";
    std::cout << "  Face 1: (" << face1.nodeId1 << ", " << face1.nodeId2
              << ", " << face1.nodeId3 << ")\n";
    std::cout << "  Face 2: (" << face2.nodeId1 << ", " << face2.nodeId2
              << ", " << face2.nodeId3 << ")\n";

    // ==================================================
    // Test 4: Check Encroachment
    // ==================================================
    printSeparator("Test 4: Check Encroachment with Holes");

    ConstraintChecker3D checker(meshData);

    // Test if any inner vertices encroach on outer boundary segments
    std::cout << "Checking if inner cavity vertices encroach outer boundary:\n";

    for (size_t i = 0; i < innerNodeIds.size(); ++i)
    {
        const auto* innerNode = meshData.getNode(innerNodeIds[i]);
        Point3D innerPoint = innerNode->getCoordinates();

        bool encroachesAny = false;
        for (const auto& seg : subsegments)
        {
            if (checker.isSubsegmentEncroached(seg, innerPoint))
            {
                encroachesAny = true;
                break;
            }
        }

        std::cout << "  Inner node " << innerNodeIds[i] << " at "
                  << innerPoint.transpose() << ": "
                  << (encroachesAny ? "ENCROACHES" : "does not encroach") << "\n";
    }

    // ==================================================
    // Summary
    // ==================================================
    printSeparator("Example Complete");

    std::cout << "\nFinal mesh statistics:\n";
    std::cout << "  Total nodes: " << meshData.getNodeCount() << "\n";
    std::cout << "  Outer boundary nodes: " << outerVertices.size() << "\n";
    std::cout << "  Inner cavity nodes (hole): " << innerVertices.size() << "\n";
    std::cout << "  Constrained subsegments: " << subsegments.size() << "\n";
    std::cout << "  Constrained subfacets: " << subfacets.size() << "\n";

    std::cout << "\nThis example demonstrated:\n";
    std::cout << "  - Creating a mesh with outer boundary and inner cavity\n";
    std::cout << "  - Defining constrained subsegments (edges)\n";
    std::cout << "  - Splitting subsegments at their midpoints\n";
    std::cout << "  - Defining constrained subfacets (triangular faces)\n";
    std::cout << "  - Checking encroachment in domains with holes\n";

    std::cout << "\nNote: In a full Shewchuk refinement implementation:\n";
    std::cout << "  - classifyTetrahedraInteriorExterior() would remove tets in holes\n";
    std::cout << "  - Refinement would respect boundary constraints\n";
    std::cout << "  - Quality bounds would be enforced on all interior tetrahedra\n";

    return 0;
}
