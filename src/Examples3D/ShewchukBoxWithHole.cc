/**
 * @file ShewchukBoxWithHole.cc
 * @brief Example demonstrating ShewchukRefiner3D on a box with a cylindrical hole
 *
 * This example demonstrates mesh generation for a more complex domain:
 * a box with a cylindrical hole drilled through it. The workflow shows:
 *
 * 1. Creating box and cylinder geometry using OpenCASCADE
 * 2. Boolean subtraction to create the hole
 * 3. Discretizing boundaries (with more samples for curved surfaces)
 * 4. Creating initial (unconstrained) Delaunay tetrahedralization
 * 5. Registering constraints for both planar and curved faces
 * 6. Applying ShewchukRefiner3D for quality mesh generation
 * 7. Exporting the refined mesh to VTK format
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
#include "spdlog/spdlog.h"

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

using namespace Meshing;

int main()
{
    spdlog::set_level(spdlog::level::info);
    spdlog::set_pattern("[%H:%M:%S] %^%v%$");

    spdlog::info("=== ShewchukRefiner3D Box with Cylindrical Hole Example ===");

    // Step 1: Create box with cylindrical hole
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

    // Step 2: Discretize boundaries
    spdlog::info("Discretizing boundary points...");

    // Use more samples for curved geometry
    Geometry3D::DiscretizationSettings3D settings(6, 3); // 6 segments/edge, 3x3 surface grid
    BoundaryDiscretizer3D discretizer(context, settings);
    auto discretization = discretizer.discretize();

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

    // Quality parameters:
    // B = 2.5 (> 2 for guaranteed termination)
    // Element limit = 10000 (more elements due to complex geometry)
    Shewchuk3DQualityController qualityController(context.getMeshData(), 2.5, 10000);

    ShewchukRefiner3D refiner(context, qualityController);
    refiner.refine();

    spdlog::info("Refined mesh: {} nodes, {} tetrahedra",
                 context.getMeshData().getNodeCount(),
                 context.getMeshData().getElementCount());

    // Step 6: Export to VTK
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
