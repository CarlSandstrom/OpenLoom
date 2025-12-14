#include <gtest/gtest.h>

#include "Common/Types.h"
#include "Geometry/Base/GeometryCollection3D.h"
#include "Meshing/Core/MeshingContext3D.h"

#define private public
#include "Meshing/Core/ConstrainedDelaunay3D.h"
#undef private

#include "Export/VtkExporter.h"
#include "Meshing/Data/MeshData3D.h"
#include "Meshing/Data/TetrahedralElement.h"
#include "Topology/Topology3D.h"
#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

TEST(Delaunay3D, SplitsSuperTetrahedronAtCenterInsertion)
{
    std::unordered_map<std::string, std::unique_ptr<Geometry3D::ISurface3D>> surfaces;
    std::unordered_map<std::string, std::unique_ptr<Geometry3D::IEdge3D>> edges;
    std::unordered_map<std::string, std::unique_ptr<Geometry3D::ICorner3D>> corners;
    Geometry3D::GeometryCollection3D geometry(std::move(surfaces), std::move(edges), std::move(corners));

    std::unordered_map<std::string, Topology3D::Surface3D> topoSurfaces;
    std::unordered_map<std::string, Topology3D::Edge3D> topoEdges;
    std::unordered_map<std::string, Topology3D::Corner3D> topoCorners;
    Topology3D::Topology3D topology(topoSurfaces, topoEdges, topoCorners);

    Meshing::MeshingContext3D context(geometry, topology);
    Meshing::ConstrainedDelaunay3D delaunay(context);

    const std::vector<Meshing::Point3D> referencePoints{
        Meshing::Point3D(-1.0, -1.0, -1.0),
        Meshing::Point3D(1.0, -1.0, -1.0),
        Meshing::Point3D(0.0, 1.0, -1.0),
        Meshing::Point3D(0.0, 0.0, 1.0)};

    delaunay.createSuperTetrahedron(referencePoints);
    EXPECT_EQ(delaunay.getActiveTetrahedronIds().size(), 1u);

    const Meshing::Point3D center(0.0, 0.0, 0.0);
    delaunay.insertVertex(center);

    //    Export::VtkExporter exporter;
    //    exporter.exportMesh(context.getMeshData(), "Delaunay3D_CenterInsertion.vtu");

    EXPECT_EQ(delaunay.getActiveTetrahedronIds().size(), 4u);
    EXPECT_EQ(context.getMeshData().getElementCount(), 4u);

    const auto& nodes = context.getMeshData().getNodes();
    size_t centerNodeId = 0;
    bool centerFound = false;
    for (const auto& [nodeId, nodePtr] : nodes)
    {
        if (nodePtr != nullptr && nodePtr->getCoordinates().isApprox(center, 1e-9))
        {
            centerNodeId = nodeId;
            centerFound = true;
            break;
        }
    }
    ASSERT_TRUE(centerFound);

    for (size_t elementId : delaunay.getActiveTetrahedronIds())
    {
        const auto* element = dynamic_cast<const Meshing::TetrahedralElement*>(context.getMeshData().getElement(elementId));
        ASSERT_NE(element, nullptr);

        const auto& nodeIds = element->getNodeIds();
        EXPECT_NE(std::find(nodeIds.begin(), nodeIds.end(), centerNodeId), nodeIds.end());
    }
}
