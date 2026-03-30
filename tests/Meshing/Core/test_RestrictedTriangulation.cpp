#include "Meshing/Core/3D/RCDT/RestrictedTriangulation.h"

#include "Common/BoundingBox2D.h"
#include "Common/Types.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Connectivity/FaceKey.h"
#include "Meshing/Core/3D/RCDT/RCDTQualitySettings.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"
#include "Meshing/Data/Base/MeshConnectivity.h"
#include "Topology/Corner3D.h"
#include "Topology/Edge3D.h"
#include "Topology/Surface3D.h"
#include "Topology/Topology3D.h"

#include <cmath>
#include <gtest/gtest.h>
#include <memory>
#include <optional>
#include <unordered_map>

using namespace Meshing;

// ============================================================================
// MockPlanarSurface — infinite plane at z = planeZ_
// ============================================================================

namespace
{

class MockPlanarSurface : public Geometry3D::ISurface3D
{
public:
    explicit MockPlanarSurface(const std::string& id, double planeZ = 0.0) :
        id_(id),
        planeZ_(planeZ)
    {
    }

    Point3D getPoint(double u, double v) const override { return Point3D(u, v, planeZ_); }

    std::array<double, 3> getNormal(double /*u*/, double /*v*/) const override
    {
        return {0.0, 0.0, 1.0};
    }

    Common::BoundingBox2D getParameterBounds() const override
    {
        return Common::BoundingBox2D(0.0, 1.0, 0.0, 1.0);
    }

    double getGap(const Point3D& point) const override { return std::abs(point.z() - planeZ_); }

    Point2D projectPoint(const Point3D& point) const override
    {
        return Point2D(point.x(), point.y());
    }

    std::optional<Point2D> projectPointToUnderlyingSurface(const Point3D& point) const override
    {
        return Point2D(point.x(), point.y());
    }

    std::optional<Point2D> projectPointToUnderlyingSurface(
        const Point3D& point, const Point2D& /*seedUV*/) const override
    {
        return projectPointToUnderlyingSurface(point);
    }

    std::string getId() const override { return id_; }

private:
    std::string id_;
    double planeZ_;
};

// ============================================================================
// Helpers to build topology and geometry for a single unnamed surface
// ============================================================================

Topology3D::Topology3D makeSingleSurfaceTopology(const std::string& surfaceId)
{
    std::unordered_map<std::string, Topology3D::Surface3D> surfaces;
    surfaces.emplace(surfaceId, Topology3D::Surface3D(surfaceId, {}, {}));

    std::unordered_map<std::string, Topology3D::Edge3D> edges;
    std::unordered_map<std::string, Topology3D::Corner3D> corners;
    return Topology3D::Topology3D(surfaces, edges, corners);
}

Geometry3D::GeometryCollection3D makeSingleSurfaceGeometry(const std::string& surfaceId,
                                                           double planeZ = 0.0)
{
    std::unordered_map<std::string, std::unique_ptr<Geometry3D::ISurface3D>> surfaces;
    surfaces[surfaceId] = std::make_unique<MockPlanarSurface>(surfaceId, planeZ);

    std::unordered_map<std::string, std::unique_ptr<Geometry3D::IEdge3D>> edges;
    std::unordered_map<std::string, std::unique_ptr<Geometry3D::ICorner3D>> corners;
    return Geometry3D::GeometryCollection3D(std::move(surfaces), std::move(edges),
                                           std::move(corners));
}

// ============================================================================
// FlatPlaneSetup
//
//  5 nodes, 2 tets sharing face {n0,n1,n2} on the z=0 plane:
//    n0=(0,0,0), n1=(1,0,0), n2=(0,1,0)  — surface nodes
//    n3=(0.33,0.33,+10)                   — interior, above
//    n4=(0.33,0.33,-10)                   — interior, below
//
//  The two tet circumcenters clearly straddle z=0, so the shared face is restricted.
// ============================================================================

struct FlatPlaneSetup
{
    static constexpr const char* SURFACE_ID = "surface";

    MeshData3D meshData;
    size_t n0, n1, n2, n3, n4;
    size_t tet0, tet1;

    Topology3D::Topology3D topology = makeSingleSurfaceTopology(SURFACE_ID);
    Geometry3D::GeometryCollection3D geometry = makeSingleSurfaceGeometry(SURFACE_ID);

    FlatPlaneSetup()
    {
        MeshMutator3D mutator(meshData);
        n0 = mutator.addBoundaryNode(Point3D(0.0, 0.0, 0.0), {}, {SURFACE_ID});
        n1 = mutator.addBoundaryNode(Point3D(1.0, 0.0, 0.0), {}, {SURFACE_ID});
        n2 = mutator.addBoundaryNode(Point3D(0.0, 1.0, 0.0), {}, {SURFACE_ID});
        n3 = mutator.addNode(Point3D(0.33, 0.33, 10.0));
        n4 = mutator.addNode(Point3D(0.33, 0.33, -10.0));

        tet0 = mutator.addElement(std::make_unique<TetrahedralElement>(
            std::array<size_t, 4>{n0, n1, n2, n3}));
        tet1 = mutator.addElement(std::make_unique<TetrahedralElement>(
            std::array<size_t, 4>{n0, n1, n2, n4}));
    }
};

} // namespace

// ============================================================================
// buildFrom: restricted face is classified correctly
// ============================================================================

TEST(RestrictedTriangulationTest, BuildFrom_RestrictsSurfaceFace)
{
    FlatPlaneSetup setup;
    MeshConnectivity connectivity(setup.meshData);

    RestrictedTriangulation rt;
    rt.buildFrom(setup.meshData, connectivity, setup.geometry, setup.topology);

    const auto& restricted = rt.getRestrictedFaces();
    ASSERT_EQ(restricted.size(), 1u);

    const FaceKey expectedFace(setup.n0, setup.n1, setup.n2);
    ASSERT_EQ(restricted.count(expectedFace), 1u);
    EXPECT_EQ(restricted.at(expectedFace), FlatPlaneSetup::SURFACE_ID);
}

// ============================================================================
// buildFrom: side faces (off-surface) are not restricted
// ============================================================================

TEST(RestrictedTriangulationTest, BuildFrom_OffSurfaceFacesNotRestricted)
{
    FlatPlaneSetup setup;
    MeshConnectivity connectivity(setup.meshData);

    RestrictedTriangulation rt;
    rt.buildFrom(setup.meshData, connectivity, setup.geometry, setup.topology);

    const auto& restricted = rt.getRestrictedFaces();

    // Side faces involve interior nodes — should not be restricted
    EXPECT_EQ(restricted.count(FaceKey(setup.n0, setup.n1, setup.n3)), 0u);
    EXPECT_EQ(restricted.count(FaceKey(setup.n1, setup.n2, setup.n3)), 0u);
    EXPECT_EQ(restricted.count(FaceKey(setup.n0, setup.n2, setup.n3)), 0u);
}

// ============================================================================
// updateAfterInsertion produces the same result as a full buildFrom rebuild
// ============================================================================

TEST(RestrictedTriangulationTest, UpdateAfterInsertion_MatchesFullRebuild)
{
    FlatPlaneSetup setup;
    MeshConnectivity connectivity(setup.meshData);

    RestrictedTriangulation rt;
    rt.buildFrom(setup.meshData, connectivity, setup.geometry, setup.topology);

    // Insert a new surface node n5 inside the original triangle
    MeshMutator3D mutator(setup.meshData);
    const size_t n5 =
        mutator.addBoundaryNode(Point3D(0.3, 0.3, 0.0), {}, {FlatPlaneSetup::SURFACE_ID});

    // Remove the original two tets and replace with six new ones
    mutator.removeElement(setup.tet0);
    mutator.removeElement(setup.tet1);

    mutator.addElement(
        std::make_unique<TetrahedralElement>(std::array<size_t, 4>{setup.n0, setup.n1, n5, setup.n3}));
    mutator.addElement(
        std::make_unique<TetrahedralElement>(std::array<size_t, 4>{setup.n1, setup.n2, n5, setup.n3}));
    mutator.addElement(
        std::make_unique<TetrahedralElement>(std::array<size_t, 4>{setup.n0, setup.n2, n5, setup.n3}));
    mutator.addElement(
        std::make_unique<TetrahedralElement>(std::array<size_t, 4>{setup.n0, setup.n1, n5, setup.n4}));
    mutator.addElement(
        std::make_unique<TetrahedralElement>(std::array<size_t, 4>{setup.n1, setup.n2, n5, setup.n4}));
    mutator.addElement(
        std::make_unique<TetrahedralElement>(std::array<size_t, 4>{setup.n0, setup.n2, n5, setup.n4}));

    connectivity.rebuildConnectivity();

    // Incremental update: the old shared face was removed from the cavity
    const std::vector<FaceKey> cavityInteriorFaces = {FaceKey(setup.n0, setup.n1, setup.n2)};
    rt.updateAfterInsertion(cavityInteriorFaces, n5, setup.meshData, connectivity,
                            setup.geometry);

    // Full rebuild on the same data for reference
    RestrictedTriangulation rtReference;
    rtReference.buildFrom(setup.meshData, connectivity, setup.geometry, setup.topology);

    // Both must agree on the restricted face set
    const auto& incremental = rt.getRestrictedFaces();
    const auto& reference = rtReference.getRestrictedFaces();

    ASSERT_EQ(incremental.size(), reference.size());
    for (const auto& [face, surfaceId] : reference)
    {
        ASSERT_EQ(incremental.count(face), 1u) << "Incremental missing a face present in reference";
        EXPECT_EQ(incremental.at(face), surfaceId);
    }
}

// ============================================================================
// getBadTriangles: elongated triangle fails circumradius/shortest-edge ratio
// ============================================================================

TEST(RestrictedTriangulationTest, GetBadTriangles_ElongatedTriangle_ReportedAsBad)
{
    // Triangle with a very small height — poor circumradius/shortest-edge ratio
    MeshData3D meshData;
    MeshMutator3D mutator(meshData);

    constexpr const char* SURFACE_ID = "surface";
    const size_t n0 = mutator.addBoundaryNode(Point3D(0.0, 0.0, 0.0), {}, {SURFACE_ID});
    const size_t n1 = mutator.addBoundaryNode(Point3D(1.0, 0.0, 0.0), {}, {SURFACE_ID});
    const size_t n2 = mutator.addBoundaryNode(Point3D(0.5, 0.01, 0.0), {}, {SURFACE_ID});
    const size_t n3 = mutator.addNode(Point3D(0.5, 0.01, 10.0));
    const size_t n4 = mutator.addNode(Point3D(0.5, 0.01, -10.0));

    mutator.addElement(
        std::make_unique<TetrahedralElement>(std::array<size_t, 4>{n0, n1, n2, n3}));
    mutator.addElement(
        std::make_unique<TetrahedralElement>(std::array<size_t, 4>{n0, n1, n2, n4}));

    MeshConnectivity connectivity(meshData);
    const auto topology = makeSingleSurfaceTopology(SURFACE_ID);
    const auto geometry = makeSingleSurfaceGeometry(SURFACE_ID);

    RestrictedTriangulation rt;
    rt.buildFrom(meshData, connectivity, geometry, topology);

    const RCDTQualitySettings settings;
    const auto badTriangles = rt.getBadTriangles(settings, meshData, geometry);

    ASSERT_FALSE(badTriangles.empty());
    const FaceKey expectedFace(n0, n1, n2);
    const bool found = std::any_of(badTriangles.begin(), badTriangles.end(),
                                   [&](const BadRestrictedTriangle& t) {
                                       return t.face == expectedFace &&
                                              t.surfaceId == SURFACE_ID;
                                   });
    EXPECT_TRUE(found);
}

// ============================================================================
// getBadTriangles: equilateral triangle passes all quality criteria
// ============================================================================

TEST(RestrictedTriangulationTest, GetBadTriangles_GoodTriangle_NotReported)
{
    MeshData3D meshData;
    MeshMutator3D mutator(meshData);

    constexpr const char* SURFACE_ID = "surface";
    const double halfBase = 0.5;
    const double height = halfBase * std::sqrt(3.0);
    const double centroidY = height / 3.0;

    const size_t n0 = mutator.addBoundaryNode(Point3D(0.0, 0.0, 0.0), {}, {SURFACE_ID});
    const size_t n1 = mutator.addBoundaryNode(Point3D(1.0, 0.0, 0.0), {}, {SURFACE_ID});
    const size_t n2 = mutator.addBoundaryNode(Point3D(halfBase, height, 0.0), {}, {SURFACE_ID});
    const size_t n3 = mutator.addNode(Point3D(halfBase, centroidY, 10.0));
    const size_t n4 = mutator.addNode(Point3D(halfBase, centroidY, -10.0));

    mutator.addElement(
        std::make_unique<TetrahedralElement>(std::array<size_t, 4>{n0, n1, n2, n3}));
    mutator.addElement(
        std::make_unique<TetrahedralElement>(std::array<size_t, 4>{n0, n1, n2, n4}));

    MeshConnectivity connectivity(meshData);
    const auto topology = makeSingleSurfaceTopology(SURFACE_ID);
    const auto geometry = makeSingleSurfaceGeometry(SURFACE_ID);

    RestrictedTriangulation rt;
    rt.buildFrom(meshData, connectivity, geometry, topology);

    const RCDTQualitySettings settings;
    const auto badTriangles = rt.getBadTriangles(settings, meshData, geometry);

    EXPECT_TRUE(badTriangles.empty());
}

// ============================================================================
// getBadTriangles: triangle off the surface fails chord deviation
// ============================================================================

TEST(RestrictedTriangulationTest, GetBadTriangles_ChordDeviationFailure)
{
    // Surface nodes are at z=0.5, but the surface is at z=0.
    // The circumcircle center is at z=0.5 — signedDistance = 0.5 > 0.1 (default max).
    MeshData3D meshData;
    MeshMutator3D mutator(meshData);

    constexpr const char* SURFACE_ID = "surface";
    constexpr double offsetZ = 0.5;

    const size_t n0 = mutator.addBoundaryNode(Point3D(0.0, 0.0, offsetZ), {}, {SURFACE_ID});
    const size_t n1 = mutator.addBoundaryNode(Point3D(1.0, 0.0, offsetZ), {}, {SURFACE_ID});
    const size_t n2 = mutator.addBoundaryNode(Point3D(0.0, 1.0, offsetZ), {}, {SURFACE_ID});
    // Interior nodes straddle z=0 so circumcenters of the two tets are on opposite sides
    const size_t n3 = mutator.addNode(Point3D(0.33, 0.33, 10.0));
    const size_t n4 = mutator.addNode(Point3D(0.33, 0.33, -10.0));

    mutator.addElement(
        std::make_unique<TetrahedralElement>(std::array<size_t, 4>{n0, n1, n2, n3}));
    mutator.addElement(
        std::make_unique<TetrahedralElement>(std::array<size_t, 4>{n0, n1, n2, n4}));

    MeshConnectivity connectivity(meshData);
    const auto topology = makeSingleSurfaceTopology(SURFACE_ID);
    // Surface stays at z=0 — nodes are offset, producing chord deviation
    const auto geometry = makeSingleSurfaceGeometry(SURFACE_ID, 0.0);

    RestrictedTriangulation rt;
    rt.buildFrom(meshData, connectivity, geometry, topology);

    // Use strict ratio so only chord deviation triggers the failure
    RCDTQualitySettings settings;
    settings.maximumCircumradiusToShortestEdgeRatio = 100.0;
    settings.maximumChordDeviation = 0.1;

    const auto badTriangles = rt.getBadTriangles(settings, meshData, geometry);

    ASSERT_FALSE(badTriangles.empty());
    const FaceKey expectedFace(n0, n1, n2);
    const bool found = std::any_of(badTriangles.begin(), badTriangles.end(),
                                   [&](const BadRestrictedTriangle& t) {
                                       return t.face == expectedFace;
                                   });
    EXPECT_TRUE(found);
}
