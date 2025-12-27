#include <gtest/gtest.h>

#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Topology/Topology3D.h"

#include "Meshing/Core/3D/MeshingContext3D.h"
#include "Meshing/Core/3D/SimpleMesher.h"

#include "Common/Types.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"

#include <array>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>

// Minimal dummy implementations of Geometry interfaces for testing
namespace Geometry3D
{

class DummySurface : public ISurface3D
{
public:
    explicit DummySurface(std::string id) :
        id_(std::move(id)) {}
    std::array<double, 3> getNormal(double, double) const override { return {0.0, 0.0, 1.0}; }
    Meshing::Point3D getPoint(double, double) const override { return Meshing::Point3D(0.0, 0.0, 0.0); }
    Common::BoundingBox2D getParameterBounds() const override
    {
        return Common::BoundingBox2D(0.0, 1.0, 0.0, 1.0);
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

class DummyEdge : public IEdge3D
{
public:
    explicit DummyEdge(std::string id) :
        id_(std::move(id)) {}
    Meshing::Point3D getPoint(double) const override { return Meshing::Point3D(0.0, 0.0, 0.0); }
    std::array<double, 3> getTangent(double) const override { return {1.0, 0.0, 0.0}; }
    Meshing::Point3D getStartPoint() const override { return Meshing::Point3D(0.0, 0.0, 0.0); }
    Meshing::Point3D getEndPoint() const override { return Meshing::Point3D(1.0, 0.0, 0.0); }
    std::pair<double, double> getParameterBounds() const override { return {0.0, 1.0}; }
    double getLength() const override { return 1.0; }
    double getCurvature(double) const override { return 0.0; }
    std::string getId() const override { return id_; }

private:
    std::string id_;
};

class DummyCorner : public ICorner3D
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

} // namespace Geometry3D

TEST(SimpleMesher, GeneratesBasicTet)
{
    // Prepare minimal geometry collection
    std::unordered_map<std::string, std::unique_ptr<Geometry3D::ISurface3D>> surfaces;
    std::unordered_map<std::string, std::unique_ptr<Geometry3D::IEdge3D>> edges;
    std::unordered_map<std::string, std::unique_ptr<Geometry3D::ICorner3D>> corners;

    surfaces["S1"] = std::make_unique<Geometry3D::DummySurface>("S1");
    edges["E1"] = std::make_unique<Geometry3D::DummyEdge>("E1");
    corners["C1"] = std::make_unique<Geometry3D::DummyCorner>("C1", Meshing::Point3D(0.0, 0.0, 0.0));
    corners["C2"] = std::make_unique<Geometry3D::DummyCorner>("C2", Meshing::Point3D(1.0, 0.0, 0.0));
    corners["C3"] = std::make_unique<Geometry3D::DummyCorner>("C3", Meshing::Point3D(0.0, 1.0, 0.0));
    corners["C4"] = std::make_unique<Geometry3D::DummyCorner>("C4", Meshing::Point3D(0.0, 0.0, 1.0));

    Geometry3D::GeometryCollection3D geom(std::move(surfaces), std::move(edges), std::move(corners));

    // Prepare empty topology (no constraints for this basic test)
    std::unordered_map<std::string, Topology3D::Surface3D> topoSurfaces;
    std::unordered_map<std::string, Topology3D::Edge3D> topoEdges;
    std::unordered_map<std::string, Topology3D::Corner3D> topoCorners;
    topoCorners.emplace("C1", Topology3D::Corner3D("C1", std::set<std::string>{}, std::set<std::string>{}));
    topoCorners.emplace("C2", Topology3D::Corner3D("C2", std::set<std::string>{}, std::set<std::string>{}));
    topoCorners.emplace("C3", Topology3D::Corner3D("C3", std::set<std::string>{}, std::set<std::string>{}));
    topoCorners.emplace("C4", Topology3D::Corner3D("C4", std::set<std::string>{}, std::set<std::string>{}));

    Topology3D::Topology3D topology(topoSurfaces, topoEdges, topoCorners);

    Meshing::MeshingContext3D context(geom, topology);
    Meshing::SimpleMesher mesher;

    mesher.generate(context);

    const auto& meshData = context.getMeshData();
    EXPECT_GE(meshData.getNodeCount(), 4u);
    EXPECT_GE(meshData.getElementCount(), 1u);
}
