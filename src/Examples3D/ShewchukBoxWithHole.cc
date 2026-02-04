/**
 * @file ShewchukBoxWithHole.cc
 * @brief Example demonstrating ShewchukRefiner3D on a box with a cylindrical hole
 *
 * This example demonstrates mesh generation for a more complex domain:
 * a box with a cylindrical hole drilled through it. The workflow shows:
 * 1. Creating box and cylinder geometry using OpenCASCADE
 * 2. Boolean subtraction to create the hole
 * 3. Discretizing boundaries (with more samples for curved surfaces)
 * 4. Setting up constraints for both planar and curved faces
 * 5. Applying ShewchukRefiner3D for quality mesh generation
 * 6. Exporting the refined mesh to VTK format
 */

#include "../Readers/OpenCascade/TopoDS_ShapeConverter.h"
#include "Export/VtkExporter.h"
#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/BoundaryDiscretizer3D.h"
#include "Meshing/Core/3D/Delaunay3D.h"
#include "Meshing/Core/3D/MeshingContext3D.h"
#include "Meshing/Core/3D/Shewchuk3DQualityController.h"
#include "Meshing/Core/3D/ShewchukRefiner3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "spdlog/spdlog.h"

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <set>

using namespace Meshing;

int main()
{
    spdlog::set_level(spdlog::level::info);
    spdlog::set_pattern("[%H:%M:%S] %^%v%$");

    spdlog::info("=== ShewchukRefiner3D Box with Cylindrical Hole Example ===");

    // =========================================
    // Step 1: Create box with cylindrical hole
    // =========================================
    spdlog::info("Creating box with cylindrical hole geometry...");

    // Create a 10x10x10 box
    TopoDS_Shape box = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();

    // Create a cylinder through the center (axis along Z, radius 2)
    gp_Pnt center(5.0, 5.0, 0.0);
    gp_Dir axisDirection(0.0, 0.0, 1.0);
    gp_Ax2 axis(center, axisDirection);
    TopoDS_Shape cylinder = BRepPrimAPI_MakeCylinder(axis, 2.0, 10.0).Shape();

    // Boolean subtraction: box - cylinder = box with hole
    TopoDS_Shape boxWithHole = BRepAlgoAPI_Cut(box, cylinder).Shape();

    spdlog::info("Box: 10x10x10, Hole: cylinder radius 2, centered at (5,5)");

    // Convert to topology and geometry
    Readers::TopoDS_ShapeConverter converter(boxWithHole);

    Meshing::MeshingContext3D context(
        converter.getGeometryCollection(),
        converter.getTopology());

    const auto* topology = context.getTopology();

    spdlog::info("Topology: {} corners, {} edges, {} surfaces",
                 topology->getAllCornerIds().size(),
                 topology->getAllEdgeIds().size(),
                 topology->getAllSurfaceIds().size());

    // =========================================
    // Step 2: Discretize boundaries
    // =========================================
    spdlog::info("Discretizing boundary points...");

    // Use more samples for curved geometry
    Geometry3D::DiscretizationSettings3D settings(6, 3); // 6 segments/edge, 3x3 surface grid
    BoundaryDiscretizer3D discretizer(context, settings);
    auto discretization = discretizer.discretize();

    spdlog::info("Total boundary points: {}", discretization.points.size());

    // =========================================
    // Step 3: Initialize Delaunay triangulation
    // =========================================
    spdlog::info("Initializing Delaunay triangulation...");

    Delaunay3D delaunay(discretization.points,
                        &context.getMeshData(),
                        discretization.edgeParameters,
                        discretization.geometryIds);
    delaunay.triangulate();

    auto pointToNodeMap = delaunay.getPointIndexToNodeIdMap();

    spdlog::info("Initial mesh: {} nodes, {} tetrahedra",
                 context.getMeshData().getNodeCount(),
                 context.getMeshData().getElementCount());

    // =========================================
    // Step 4: Set up constraints
    // =========================================
    spdlog::info("Setting up boundary constraints...");

    auto& mutator = context.getMutator();

    // Create subsegments for each edge
    for (const auto& edgeId : topology->getAllEdgeIds())
    {
        const auto& edgePointIndices = discretization.edgeIdToPointIndicesMap.at(edgeId);

        for (size_t i = 0; i + 1 < edgePointIndices.size(); ++i)
        {
            size_t nodeId1 = pointToNodeMap.at(edgePointIndices[i]);
            size_t nodeId2 = pointToNodeMap.at(edgePointIndices[i + 1]);

            ConstrainedSubsegment3D seg;
            seg.nodeId1 = nodeId1;
            seg.nodeId2 = nodeId2;
            seg.geometryId = edgeId;
            mutator.addConstrainedSubsegment(seg);
        }
    }

    // Create subfacets for each surface
    for (const auto& surfaceId : topology->getAllSurfaceIds())
    {
        const auto& topoSurface = topology->getSurface(surfaceId);
        const auto& boundaryEdgeIds = topoSurface.getBoundaryEdgeIds();

        // Collect all boundary nodes for this face
        std::vector<size_t> faceNodes;
        std::set<size_t> seenNodes;

        for (const auto& edgeId : boundaryEdgeIds)
        {
            const auto& chain = discretization.edgeIdToPointIndicesMap.at(edgeId);
            for (size_t pointIdx : chain)
            {
                size_t nodeId = pointToNodeMap.at(pointIdx);
                if (seenNodes.find(nodeId) == seenNodes.end())
                {
                    faceNodes.push_back(nodeId);
                    seenNodes.insert(nodeId);
                }
            }
        }

        // Simple fan triangulation from first vertex
        if (faceNodes.size() >= 3)
        {
            for (size_t i = 1; i + 1 < faceNodes.size(); ++i)
            {
                ConstrainedSubfacet3D facet;
                facet.nodeId1 = faceNodes[0];
                facet.nodeId2 = faceNodes[i];
                facet.nodeId3 = faceNodes[i + 1];
                facet.geometryId = surfaceId;
                mutator.addConstrainedSubfacet(facet);
            }
        }
    }

    spdlog::info("Constraints: {} subsegments, {} subfacets",
                 context.getMeshData().getConstrainedSubsegmentCount(),
                 context.getMeshData().getConstrainedSubfacetCount());

    // =========================================
    // Step 5: Apply ShewchukRefiner3D
    // =========================================
    spdlog::info("Starting Shewchuk refinement...");

    // Quality parameters:
    // B = 2.5 (> 2 for guaranteed termination)
    // Element limit = 10000 (more elements due to complex geometry)
    Shewchuk3DQualityController qualityController(context.getMeshData(), 2.5, 10000);

    ShewchukRefiner3D refiner(context, qualityController);
    refiner.refine();

    spdlog::info("Refined mesh: {} nodes, {} tetrahedra",
                 context.getMeshData().getNodeCount(),
                 context.getMeshData().getElementCount());

    // =========================================
    // Step 6: Export to VTK
    // =========================================
    spdlog::info("Exporting mesh to shewchuk_box_with_hole.vtu...");

    Export::VtkExporter exporter;
    exporter.writeVtu(context.getMeshData(), "shewchuk_box_with_hole.vtu");

    spdlog::info("=== Example Complete ===");
    spdlog::info("View in ParaView: paraview shewchuk_box_with_hole.vtu");
    spdlog::info("");
    spdlog::info("Notes:");
    spdlog::info("  - The cylindrical hole creates curved boundary surfaces");
    spdlog::info("  - More samples are used for curved edges and surfaces");
    spdlog::info("  - The refiner respects both planar and curved constraints");

    return 0;
}
