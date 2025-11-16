#include <fstream>
#include <gtest/gtest.h>
#include <set>
#include <string>

#include "Export/VtkExporter.h"
#include "Meshing/Core/MeshingContext.h"
#include "Meshing/Core/SimpleMesher.h"

#include "Common/Types.h"
#include "Geometry/Base/GeometryCollection.h"
#include "Topology/Topology.h"

// Dummy geometry implementations (reuse from SimpleMesher test if needed)
namespace Geometry
{
class DummySurface : public Surface
{
public:
    explicit DummySurface(std::string id) :
        id_(std::move(id)) {}
    std::array<double, 3> getNormal(double, double) const override { return {0.0, 0.0, 1.0}; }
    Meshing::Point3D getPoint(double, double) const override { return Meshing::Point3D(0.0, 0.0, 0.0); }
    void getParameterBounds(double& uMin, double& uMax, double& vMin, double& vMax) const override
    {
        uMin = 0.0;
        uMax = 1.0;
        vMin = 0.0;
        vMax = 1.0;
    }
    double getGap(const Meshing::Point3D&) const override { return 0.0; }
    Meshing::Point2D projectPoint(const Meshing::Point3D&) const override
    {
        return Meshing::Point2D(0.0, 0.0);
    }
    std::string getId() const override { return id_; }

private:
    std::string id_;
};
class DummyEdge : public Edge
{
public:
    explicit DummyEdge(std::string id) :
        id_(std::move(id)) {}
    Meshing::Point3D getPoint(double) const override { return Meshing::Point3D(0.0, 0.0, 0.0); }
    std::array<double, 3> getTangent(double) const override { return {1.0, 0.0, 0.0}; }
    Meshing::Point3D getStartPoint() const override { return Meshing::Point3D(0.0, 0.0, 0.0); }
    Meshing::Point3D getEndPoint() const override { return Meshing::Point3D(1.0, 0.0, 0.0); }
    void getParameterBounds(double& tMin, double& tMax) const override
    {
        tMin = 0.0;
        tMax = 1.0;
    }
    double getLength() const override { return 1.0; }
    double getCurvature(double) const override { return 0.0; }
    std::string getId() const override { return id_; }

private:
    std::string id_;
};
class DummyCorner : public Corner
{
public:
    explicit DummyCorner(std::string id, Meshing::Point3D p = Meshing::Point3D(0.0, 0.0, 0.0)) :
        id_(std::move(id)), p_(p) {}
    Meshing::Point3D getPoint() const override { return p_; }
    std::string getId() const override { return id_; }

private:
    std::string id_;
    Meshing::Point3D p_;
};
} // namespace Geometry

TEST(VtkExporter, WritesBasicVtu)
{
    // Setup minimal geometry & topology
    std::unordered_map<std::string, std::unique_ptr<Geometry::Surface>> surfaces;
    std::unordered_map<std::string, std::unique_ptr<Geometry::Edge>> edges;
    std::unordered_map<std::string, std::unique_ptr<Geometry::Corner>> corners;
    surfaces["S1"] = std::make_unique<Geometry::DummySurface>("S1");
    edges["E1"] = std::make_unique<Geometry::DummyEdge>("E1");
    corners["C1"] = std::make_unique<Geometry::DummyCorner>("C1", Meshing::Point3D(0.0, 0.0, 0.0));
    corners["C2"] = std::make_unique<Geometry::DummyCorner>("C2", Meshing::Point3D(1.0, 0.0, 0.0));
    corners["C3"] = std::make_unique<Geometry::DummyCorner>("C3", Meshing::Point3D(0.0, 1.0, 0.0));
    corners["C4"] = std::make_unique<Geometry::DummyCorner>("C4", Meshing::Point3D(0.0, 0.0, 1.0));

    Geometry::GeometryCollection geom(std::move(surfaces), std::move(edges), std::move(corners));

    std::unordered_map<std::string, Topology::Surface> topoSurfaces;
    std::unordered_map<std::string, Topology::Edge> topoEdges;
    std::unordered_map<std::string, Topology::Corner> topoCorners;
    topoCorners.emplace("C1", Topology::Corner("C1", std::set<std::string>{}, std::set<std::string>{}));
    topoCorners.emplace("C2", Topology::Corner("C2", std::set<std::string>{}, std::set<std::string>{}));
    topoCorners.emplace("C3", Topology::Corner("C3", std::set<std::string>{}, std::set<std::string>{}));
    topoCorners.emplace("C4", Topology::Corner("C4", std::set<std::string>{}, std::set<std::string>{}));
    Topology::Topology topology(topoSurfaces, topoEdges, topoCorners);

    Meshing::MeshingContext context(geom, topology);
    Meshing::SimpleMesher mesher; // generates a single tetra
    mesher.generate(context);

    Export::VtkExporter exporter;
    const std::string filePath = "test_output.vtu";
    ASSERT_TRUE(exporter.exportMesh(context.getMeshData(), filePath));

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
