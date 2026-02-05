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
    spdlog::set_level(spdlog::level::trace);
    spdlog::set_pattern("[%H:%M:%S] %s: %^%v%$");

    TopoDS_Shape cube = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();
    Readers::TopoDS_ShapeConverter converter(cube);
    MeshingContext3D context(
        converter.getGeometryCollection(),
        converter.getTopology());

    const auto* topology = context.getTopology();

    // Discretize boundaries
    Geometry3D::DiscretizationSettings3D settings(3, 2);
    BoundaryDiscretizer3D discretizer(context, settings);
    auto discretization = discretizer.discretize();

    // Create initial Delaunay triangulation
    Delaunay3D delaunay(discretization.points,
                        &context.getMeshData(),
                        discretization.edgeParameters,
                        discretization.geometryIds);
    delaunay.triangulate();

    auto pointToNodeMap = delaunay.getPointIndexToNodeIdMap();

    // Set up constraint subsegments for each edge
    auto& mutator = context.getMutator();

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

    // Set up constraint subfacets for each surface
    for (const auto& surfaceId : topology->getAllSurfaceIds())
    {
        const auto& topoSurface = topology->getSurface(surfaceId);
        const auto& boundaryEdgeIds = topoSurface.getBoundaryEdgeIds();

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
