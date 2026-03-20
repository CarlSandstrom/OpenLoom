/**
 * @file ShewchukBox.cc
 * @brief Example demonstrating ShewchukRefiner3D on a simple box
 *
 * This example shows the workflow for generating a quality tetrahedral mesh
 * of a box using Shewchuk's Delaunay refinement algorithm:
 *
 * 1. Create box geometry using OpenCASCADE
 * 2. Discretize boundaries using BoundaryDiscretizer3D
 * 3. Create initial (unconstrained) Delaunay triangulation using Delaunay3D
 * 4. Register constraints (subsegments and subfacets) - may not appear in mesh yet
 * 5. Apply ShewchukRefiner3D to recover constraints and improve mesh quality
 * 6. Export the refined mesh to VTK format
 *
 * Per Shewchuk's algorithm, the initial Delaunay tetrahedralization may not
 * contain all constraint edges/faces. The refiner recovers missing constraints
 * by splitting encroached subsegments and subfacets.
 */

#include "../Readers/OpenCascade/TopoDS_ShapeConverter.h"
#include "Export/VtkExporter.h"
#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/General/BoundaryDiscretizer3D.h"
#include "Meshing/Core/3D/General/ConstraintRegistrar3D.h"
#include "Meshing/Core/3D/Volume/Delaunay3D.h"
#include "Meshing/Core/3D/General/MeshingContext3D.h"
#include "Meshing/Core/3D/Volume/Shewchuk3DQualityController.h"
#include "Meshing/Core/3D/Volume/ShewchukRefiner3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Common/Logging.h"
#include "spdlog/spdlog.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <TopoDS_Shape.hxx>

using namespace Meshing;

int main()
{
    Common::initLogging();

    spdlog::info("=== ShewchukRefiner3D Box Example ===");

    // Step 1: Create box geometry with OpenCASCADE
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

    // Step 2: Discretize boundaries
    spdlog::info("Discretizing boundary points...");

    Geometry3D::DiscretizationSettings3D settings(3, 2); // 3 segments/edge, 2x2 surface grid
    BoundaryDiscretizer3D discretizer(*context.getGeometry(), *context.getTopology(), settings);
    discretizer.discretize();
    const auto& discretization = discretizer.getDiscretizationResult();

    spdlog::info("Total boundary points: {}", discretization.points.size());

    // Step 3: Create initial (unconstrained) Delaunay tetrahedralization
    spdlog::info("Creating Delaunay tetrahedralization...");

    Delaunay3D delaunay(discretization.points,
                        &context.getMeshData(),
                        discretization.edgeParameters,
                        discretization.geometryIds);
    delaunay.triangulate();

    spdlog::info("Initial mesh: {} nodes, {} tetrahedra",
                 context.getMeshData().getNodeCount(),
                 context.getMeshData().getElementCount());

    // Step 4: Register constraints (subsegments and subfacets)
    // These define what edges/faces *should* appear in the final mesh.
    // Some may be missing from the current tetrahedralization.
    spdlog::info("Registering boundary constraints...");

    ConstraintRegistrar3D registrar(context, discretization);
    registrar.registerConstraints(delaunay.getPointIndexToNodeIdMap());

    // Step 5: Apply ShewchukRefiner3D
    // This recovers missing constraints and improves mesh quality.
    spdlog::info("Starting Shewchuk refinement...");

    // Quality parameters: B = 2.5 (> 2 for guaranteed termination), max 5000 elements
    Shewchuk3DQualityController qualityController(context.getMeshData(), 2.5, 5000);

    ShewchukRefiner3D refiner(context, qualityController);
    refiner.refine();

    spdlog::info("Refined mesh: {} nodes, {} tetrahedra",
                 context.getMeshData().getNodeCount(),
                 context.getMeshData().getElementCount());

    // Step 6: Export to VTK
    spdlog::info("Exporting mesh to shewchuk_box.vtu...");

    Export::VtkExporter exporter;
    exporter.writeVtu(context.getMeshData(), "shewchuk_box.vtu");

    spdlog::info("=== Example Complete ===");
    spdlog::info("View in ParaView: paraview shewchuk_box.vtu");

    return 0;
}
