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

#include <BRepPrimAPI_MakeBox.hxx>
#include <TopoDS_Shape.hxx>

using namespace Meshing;

int main()
{
    spdlog::set_level(spdlog::level::trace);
    spdlog::set_pattern("[%H:%M:%S] %s: %^%v%$");

    TopoDS_Shape cube = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();
    Readers::TopoDS_ShapeConverter converter(cube);
    MeshingContext3D context(
        converter.getGeometryCollection(),
        converter.getTopology());

    // Discretize boundaries
    Geometry3D::DiscretizationSettings3D settings(3, 2);
    BoundaryDiscretizer3D discretizer(context, settings);
    auto discretization = discretizer.discretize();

    // Create initial (unconstrained) Delaunay tetrahedralization
    Delaunay3D delaunay(discretization.points,
                        &context.getMeshData(),
                        discretization.edgeParameters,
                        discretization.geometryIds);
    delaunay.triangulate();

    // Register constraints (subsegments and subfacets)
    ConstraintRegistrar3D registrar(context, discretization);
    registrar.registerConstraints(delaunay.getPointIndexToNodeIdMap());

    // Refine with Shewchuk quality control
    Shewchuk3DQualityController qualityController(
        context.getMeshData(),
        2.5,  // Min radius-edge ratio
        10000 // Max elements
    );

    ShewchukRefiner3D refiner(context, qualityController);
    refiner.refine();

    Export::VtkExporter exporter;
    exporter.writeVtu(context.getMeshData(), "box_mesh.vtu");

    return 0;
}
