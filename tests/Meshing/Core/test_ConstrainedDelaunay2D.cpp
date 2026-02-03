#include <gtest/gtest.h>

#include "Common/Types.h"
#include "Geometry/2D/Base/Corner2D.h"
#include "Geometry/2D/Base/DiscretizationSettings2D.h"
#include "Geometry/2D/Base/GeometryCollection2D.h"
#include "Geometry/2D/Base/LinearEdge2D.h"
#include "Meshing/Core/2D/ConstrainedDelaunay2D.h"
#include "Meshing/Core/2D/EdgeDiscretizer2D.h"
#include "Meshing/Core/2D/MeshingContext2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Topology2D/Corner2D.h"
#include "Topology2D/Edge2D.h"
#include "Topology2D/Topology2D.h"

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

using namespace Meshing;

namespace
{

/**
 * Helper to build a square MeshingContext2D with corners at (0,0), (1,0), (1,1), (0,1).
 */
MeshingContext2D makeSquareContext()
{
    Point2D p0(0.0, 0.0);
    Point2D p1(1.0, 0.0);
    Point2D p2(1.0, 1.0);
    Point2D p3(0.0, 1.0);

    auto geometry = std::make_unique<Geometry2D::GeometryCollection2D>();
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("c0", p0));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("c1", p1));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("c2", p2));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("c3", p3));

    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("e0", p0, p1));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("e1", p1, p2));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("e2", p2, p3));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("e3", p3, p0));

    std::unordered_map<std::string, Topology2D::Corner2D> topoCorners;
    topoCorners.emplace("c0", Topology2D::Corner2D("c0", std::set<std::string>{"e3", "e0"}));
    topoCorners.emplace("c1", Topology2D::Corner2D("c1", std::set<std::string>{"e0", "e1"}));
    topoCorners.emplace("c2", Topology2D::Corner2D("c2", std::set<std::string>{"e1", "e2"}));
    topoCorners.emplace("c3", Topology2D::Corner2D("c3", std::set<std::string>{"e2", "e3"}));

    std::unordered_map<std::string, Topology2D::Edge2D> topoEdges;
    topoEdges.emplace("e0", Topology2D::Edge2D("e0", "c0", "c1"));
    topoEdges.emplace("e1", Topology2D::Edge2D("e1", "c1", "c2"));
    topoEdges.emplace("e2", Topology2D::Edge2D("e2", "c2", "c3"));
    topoEdges.emplace("e3", Topology2D::Edge2D("e3", "c3", "c0"));

    std::vector<std::string> boundaryEdgeLoop = {"e0", "e1", "e2", "e3"};

    auto topology = std::make_unique<Topology2D::Topology2D>(topoCorners, topoEdges, boundaryEdgeLoop);

    return MeshingContext2D(std::move(geometry), std::move(topology));
}

/**
 * Helper to build a triangular MeshingContext2D with corners at (0,0), (1,0), (0.5,1).
 */
MeshingContext2D makeTriangleContext()
{
    Point2D p0(0.0, 0.0);
    Point2D p1(1.0, 0.0);
    Point2D p2(0.5, 1.0);

    auto geometry = std::make_unique<Geometry2D::GeometryCollection2D>();
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("c0", p0));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("c1", p1));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("c2", p2));

    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("e0", p0, p1));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("e1", p1, p2));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("e2", p2, p0));

    std::unordered_map<std::string, Topology2D::Corner2D> topoCorners;
    topoCorners.emplace("c0", Topology2D::Corner2D("c0", std::set<std::string>{"e2", "e0"}));
    topoCorners.emplace("c1", Topology2D::Corner2D("c1", std::set<std::string>{"e0", "e1"}));
    topoCorners.emplace("c2", Topology2D::Corner2D("c2", std::set<std::string>{"e1", "e2"}));

    std::unordered_map<std::string, Topology2D::Edge2D> topoEdges;
    topoEdges.emplace("e0", Topology2D::Edge2D("e0", "c0", "c1"));
    topoEdges.emplace("e1", Topology2D::Edge2D("e1", "c1", "c2"));
    topoEdges.emplace("e2", Topology2D::Edge2D("e2", "c2", "c0"));

    std::vector<std::string> boundaryEdgeLoop = {"e0", "e1", "e2"};

    auto topology = std::make_unique<Topology2D::Topology2D>(topoCorners, topoEdges, boundaryEdgeLoop);

    return MeshingContext2D(std::move(geometry), std::move(topology));
}

/**
 * Helper to build an L-shaped MeshingContext2D.
 * Corners: (0,0), (1,0), (1,0.5), (0.5,0.5), (0.5,1), (0,1)
 */
MeshingContext2D makeLShapeContext()
{
    Point2D p0(0.0, 0.0);
    Point2D p1(1.0, 0.0);
    Point2D p2(1.0, 0.5);
    Point2D p3(0.5, 0.5);
    Point2D p4(0.5, 1.0);
    Point2D p5(0.0, 1.0);

    auto geometry = std::make_unique<Geometry2D::GeometryCollection2D>();
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("c0", p0));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("c1", p1));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("c2", p2));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("c3", p3));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("c4", p4));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("c5", p5));

    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("e0", p0, p1));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("e1", p1, p2));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("e2", p2, p3));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("e3", p3, p4));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("e4", p4, p5));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("e5", p5, p0));

    std::unordered_map<std::string, Topology2D::Corner2D> topoCorners;
    topoCorners.emplace("c0", Topology2D::Corner2D("c0", std::set<std::string>{"e5", "e0"}));
    topoCorners.emplace("c1", Topology2D::Corner2D("c1", std::set<std::string>{"e0", "e1"}));
    topoCorners.emplace("c2", Topology2D::Corner2D("c2", std::set<std::string>{"e1", "e2"}));
    topoCorners.emplace("c3", Topology2D::Corner2D("c3", std::set<std::string>{"e2", "e3"}));
    topoCorners.emplace("c4", Topology2D::Corner2D("c4", std::set<std::string>{"e3", "e4"}));
    topoCorners.emplace("c5", Topology2D::Corner2D("c5", std::set<std::string>{"e4", "e5"}));

    std::unordered_map<std::string, Topology2D::Edge2D> topoEdges;
    topoEdges.emplace("e0", Topology2D::Edge2D("e0", "c0", "c1"));
    topoEdges.emplace("e1", Topology2D::Edge2D("e1", "c1", "c2"));
    topoEdges.emplace("e2", Topology2D::Edge2D("e2", "c2", "c3"));
    topoEdges.emplace("e3", Topology2D::Edge2D("e3", "c3", "c4"));
    topoEdges.emplace("e4", Topology2D::Edge2D("e4", "c4", "c5"));
    topoEdges.emplace("e5", Topology2D::Edge2D("e5", "c5", "c0"));

    std::vector<std::string> boundaryEdgeLoop = {"e0", "e1", "e2", "e3", "e4", "e5"};

    auto topology = std::make_unique<Topology2D::Topology2D>(topoCorners, topoEdges, boundaryEdgeLoop);

    return MeshingContext2D(std::move(geometry), std::move(topology));
}

} // namespace

TEST(ConstrainedDelaunay2D, TriangulatesSimpleSquare)
{
    auto context = makeSquareContext();

    EdgeDiscretizer2D discretizer(context);
    auto discretization = discretizer.discretize();

    ConstrainedDelaunay2D delaunay(context, discretization);
    delaunay.triangulate();

    const auto& meshData = context.getMeshData();

    // A square with 4 corner points should produce 2 triangles
    EXPECT_EQ(meshData.getElementCount(), 2u);
    EXPECT_GE(meshData.getNodeCount(), 4u);

    // Verify all elements have 3 nodes (triangles)
    for (const auto& [id, element] : meshData.getElements())
    {
        EXPECT_EQ(element->getNodeCount(), 3u);
    }
}

TEST(ConstrainedDelaunay2D, TriangulatesSquareWithInteriorPoint)
{
    auto context = makeSquareContext();

    EdgeDiscretizer2D discretizer(context);
    auto discretization = discretizer.discretize();

    std::vector<Point2D> additionalPoints = {Point2D(0.5, 0.5)};
    ConstrainedDelaunay2D delaunay(context, discretization, additionalPoints);
    delaunay.triangulate();

    const auto& meshData = context.getMeshData();

    // A square with one interior point should create 4 triangles
    EXPECT_EQ(meshData.getElementCount(), 4u);
    EXPECT_GE(meshData.getNodeCount(), 5u);
}

TEST(ConstrainedDelaunay2D, HandlesConcavePolygon)
{
    auto context = makeLShapeContext();

    EdgeDiscretizer2D discretizer(context);
    auto discretization = discretizer.discretize();

    ConstrainedDelaunay2D delaunay(context, discretization);
    delaunay.triangulate();

    const auto& meshData = context.getMeshData();

    // L-shape with 6 corners: should produce 4 triangles
    EXPECT_EQ(meshData.getElementCount(), 4u);
    EXPECT_GE(meshData.getNodeCount(), 6u);

    // Verify all elements are triangles
    for (const auto& [id, element] : meshData.getElements())
    {
        EXPECT_EQ(element->getNodeCount(), 3u);
    }
}

TEST(ConstrainedDelaunay2D, TriangulatesTriangle)
{
    auto context = makeTriangleContext();

    EdgeDiscretizer2D discretizer(context);
    auto discretization = discretizer.discretize();

    ConstrainedDelaunay2D delaunay(context, discretization);
    delaunay.triangulate();

    const auto& meshData = context.getMeshData();

    // Should create exactly 1 triangle
    EXPECT_EQ(meshData.getElementCount(), 1u);
    EXPECT_GE(meshData.getNodeCount(), 3u);

    // Verify the single element has 3 nodes
    const auto& [id, element] = *meshData.getElements().begin();
    EXPECT_EQ(element->getNodeCount(), 3u);
}
