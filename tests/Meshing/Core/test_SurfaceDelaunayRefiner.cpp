#include "Meshing/Core/3D/RCDT/SurfaceDelaunayRefiner.h"

#include "Meshing/Core/3D/General/MeshingContext3D.h"
#include "Meshing/Core/3D/RCDT/SurfaceFacetCriteria.h"
#include "Meshing/Core/3D/RCDT/WeightedDualRestriction.h"
#include "Meshing/Core/3D/Volume/Delaunay3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Readers/OpenCascade/TopoDS_ShapeConverter.h"
#include "Topology/Topology3D.h"

#include <BRepPrimAPI_MakeSphere.hxx>

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <numbers>
#include <set>
#include <string>
#include <vector>

#include <gtest/gtest.h>

using namespace Meshing;

// ============================================================================
// SurfaceDelaunayRefinerTest
//
// The CGAL-style path on the case its theory covers: a coarse sample of a
// smooth closed surface, refined until no facet fails a criterion. Delaunay
// refinement on a smooth surface terminates with every facet meeting the
// angle bound, and the restricted facets of the result triangulate the
// surface. 40 evenly spread points on a sphere of radius 3 already meet the
// angle bound; a distance bound of 0.05 (their facets sag about 0.17) is what
// gives refinement real work to do.
// ============================================================================

class SurfaceDelaunayRefinerTest : public ::testing::Test
{
protected:
    static constexpr double RADIUS = 3.0;
    static constexpr size_t SAMPLE_COUNT = 40;
    static constexpr double MINIMUM_EDGE_LENGTH = 0.1;
    static constexpr double MINIMUM_ANGLE_DEGREES = 30.0;

    static void SetUpTestSuite()
    {
        converter_ = std::make_unique<Readers::TopoDS_ShapeConverter>(BRepPrimAPI_MakeSphere(RADIUS).Shape());
        const auto& topology = converter_->getTopology();
        const std::string surfaceId = topology.getAllSurfaceIds().front();

        std::vector<Point3D> points;
        std::vector<std::vector<std::string>> geometryIds;
        const double goldenAngle = std::numbers::pi * (3.0 - std::sqrt(5.0));
        for (size_t i = 0; i < SAMPLE_COUNT; ++i)
        {
            const double z = 1.0 - 2.0 * (static_cast<double>(i) + 0.5) / static_cast<double>(SAMPLE_COUNT);
            const double ring = std::sqrt(1.0 - z * z);
            const double angle = goldenAngle * static_cast<double>(i);
            points.emplace_back(RADIUS * ring * std::cos(angle), RADIUS * ring * std::sin(angle), RADIUS * z);
            geometryIds.push_back({surfaceId});
        }

        context_ = std::make_unique<MeshingContext3D>(converter_->getGeometryCollection(), topology);
        Delaunay3D::triangulate(context_->getOperations(), points, geometryIds);

        SurfaceMesh3DQualitySettings settings;
        settings.minAngleDegrees = MINIMUM_ANGLE_DEGREES;
        settings.chordDeviationTolerance = 0.05;
        settings.maxRefinementIterations = 5000;
        refiner_ = std::make_unique<SurfaceDelaunayRefiner>(*context_, topology, settings, MINIMUM_EDGE_LENGTH);
        refiner_->refine();
        faces_ = refiner_->getRestrictedFaces();
    }

    static void TearDownTestSuite()
    {
        refiner_.reset();
        context_.reset();
        converter_.reset();
    }

    static std::unique_ptr<Readers::TopoDS_ShapeConverter> converter_;
    static std::unique_ptr<MeshingContext3D> context_;
    static std::unique_ptr<SurfaceDelaunayRefiner> refiner_;
    static RestrictedFaceMap faces_;
};

std::unique_ptr<Readers::TopoDS_ShapeConverter> SurfaceDelaunayRefinerTest::converter_;
std::unique_ptr<MeshingContext3D> SurfaceDelaunayRefinerTest::context_;
std::unique_ptr<SurfaceDelaunayRefiner> SurfaceDelaunayRefinerTest::refiner_;
RestrictedFaceMap SurfaceDelaunayRefinerTest::faces_;

TEST_F(SurfaceDelaunayRefinerTest, TerminatesWithoutDroppingAnyFacet)
{
    EXPECT_GT(refiner_->getInsertionCount(), 0u);
    EXPECT_LT(refiner_->getInsertionCount(), 5000u);
    EXPECT_EQ(refiner_->getDroppedCount(), 0u);
}

TEST_F(SurfaceDelaunayRefinerTest, RestrictedFacetsFormAClosedGenusZeroSurface)
{
    ASSERT_FALSE(faces_.empty());
    std::map<std::pair<size_t, size_t>, int> edgeFaceCount;
    std::set<size_t> vertices;
    for (const auto& [face, surfaceId] : faces_)
    {
        const auto& n = face.nodeIds;
        for (const auto& [a, b] : {std::make_pair(n[0], n[1]), std::make_pair(n[0], n[2]), std::make_pair(n[1], n[2])})
            ++edgeFaceCount[{std::min(a, b), std::max(a, b)}];
        vertices.insert(n.begin(), n.end());
    }
    for (const auto& [edge, count] : edgeFaceCount)
        EXPECT_EQ(count, 2) << "edge " << edge.first << "-" << edge.second;
    EXPECT_EQ(static_cast<long>(vertices.size()) - static_cast<long>(edgeFaceCount.size()) +
                  static_cast<long>(faces_.size()),
              2);
}

TEST_F(SurfaceDelaunayRefinerTest, EveryFacetMeetsTheAngleBoundAndEveryVertexIsOnTheSphere)
{
    const auto& meshData = context_->getMeshData();
    for (const auto& [face, surfaceId] : faces_)
    {
        std::array<Point3D, 3> p;
        for (int i = 0; i < 3; ++i)
        {
            p[i] = meshData.getNode(face.nodeIds[i])->getCoordinates();
            EXPECT_NEAR(p[i].norm(), RADIUS, 1e-6);
        }
        double smallestAngle = 180.0;
        for (int i = 0; i < 3; ++i)
        {
            const Point3D u = p[(i + 1) % 3] - p[i];
            const Point3D v = p[(i + 2) % 3] - p[i];
            smallestAngle = std::min(smallestAngle, std::acos(u.normalized().dot(v.normalized())) * 180.0 / std::numbers::pi);
        }
        EXPECT_GE(smallestAngle, MINIMUM_ANGLE_DEGREES - 1e-6);
    }
}

// CGAL's same-patch rule compares surface-interior vertices with each other
// only: a vertex on two surfaces' shared curve does not count against either.
TEST(SurfaceFacetCriteriaTest, SamePatchRuleFlagsOnlySurfaceInteriorVerticesOnDifferentSurfaces)
{
    MeshData3D meshData;
    MeshMutator3D mutator(meshData);
    const size_t onTop = mutator.addBoundaryNode(Point3D(0.0, 0.0, 1.0), {"top"});
    const size_t onSide = mutator.addBoundaryNode(Point3D(1.0, 0.0, 0.0), {"side"});
    const size_t onCurve = mutator.addBoundaryNode(Point3D(0.0, 1.0, 0.0), {"crease"});
    const size_t alsoOnTop = mutator.addBoundaryNode(Point3D(1.0, 1.0, 1.0), {"top"});

    SurfaceMesh3DQualitySettings settings;
    settings.minAngleDegrees = 0.0;
    settings.chordDeviationTolerance = 0.0;
    const SurfaceFacetCriteria criteria(settings, {"top", "side"});
    const RestrictedFacet facet{"top", Point3D::Zero()};

    const auto straddling = criteria.findBadness(FaceKey(onTop, onSide, onCurve), facet, meshData);
    ASSERT_TRUE(straddling.has_value());
    EXPECT_EQ(straddling->first, 2);

    EXPECT_FALSE(criteria.findBadness(FaceKey(onTop, alsoOnTop, onCurve), facet, meshData).has_value());
}
