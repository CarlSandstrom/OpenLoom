/**
 * @file ShewchukBox.cc
 * @brief Example demonstrating ShewchukRefiner3D on a simple box
 *
 * This example shows the complete workflow for generating a quality
 * tetrahedral mesh of a box using Shewchuk's Delaunay refinement algorithm:
 * 1. Create box geometry using OpenCASCADE
 * 2. Discretize boundaries using BoundaryDiscretizer3D
 * 3. Create initial Delaunay triangulation using Delaunay3D
 * 4. Set up constraint subsegments (edges) and subfacets (faces)
 * 5. Apply ShewchukRefiner3D to improve mesh quality
 * 6. Export the refined mesh to VTK format
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

#include <BRepPrimAPI_MakeBox.hxx>
#include <TopoDS_Shape.hxx>

#include <set>

using namespace Meshing;

int main()
{
    spdlog::set_level(spdlog::level::info);
    spdlog::set_pattern("[%H:%M:%S] %^%v%$");

    spdlog::info("=== ShewchukRefiner3D Box Example ===");

    // =========================================
    // Step 1: Create box geometry with OpenCASCADE
    // =========================================
    spdlog::info("Creating 10x10x10 box geometry...");

    TopoDS_Shape box = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();
    Readers::TopoDS_ShapeConverter converter(box);

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

    Geometry3D::DiscretizationSettings3D settings(3, 2); // 3 segments/edge, 2x2 surface grid
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
    // Step 4: Set up constraints (subsegments and subfacets)
    // =========================================
    spdlog::info("Setting up boundary constraints...");

    auto& mutator = context.getMutator();

    // Create subsegments for each edge
    for (const auto& edgeId : topology->getAllEdgeIds())
    {
        const auto& edgePointIndices = discretization.edgeIdToPointIndicesMap.at(edgeId);

        // Create subsegments connecting consecutive points along the edge
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

        // Collect all boundary nodes for this face, avoiding duplicates
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

    // Quality parameters: B = 2.5 (> 2 for guaranteed termination), max 5000 elements
    Shewchuk3DQualityController qualityController(context.getMeshData(), 2.5, 5000);

    ShewchukRefiner3D refiner(context, qualityController);
    refiner.refine();

    spdlog::info("Refined mesh: {} nodes, {} tetrahedra",
                 context.getMeshData().getNodeCount(),
                 context.getMeshData().getElementCount());

    // =========================================
    // Step 6: Export to VTK
    // =========================================
    spdlog::info("Exporting mesh to shewchuk_box.vtu...");

    Export::VtkExporter exporter;
    exporter.writeVtu(context.getMeshData(), "shewchuk_box.vtu");

    spdlog::info("=== Example Complete ===");
    spdlog::info("View in ParaView: paraview shewchuk_box.vtu");

    return 0;
}
