#include "Meshing/Core/3D/RCDT/SurfaceDelaunayRefiner.h"

#include "Meshing/Core/3D/General/MeshSnapshotWriter.h"
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
#include <cstdlib>
#include <filesystem>
#include <fstream>
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

    static void sampleSphere(const std::string& surfaceId,
                             std::vector<Point3D>& points,
                             std::vector<std::vector<std::string>>& geometryIds)
    {
        const double goldenAngle = std::numbers::pi * (3.0 - std::sqrt(5.0));
        for (size_t i = 0; i < SAMPLE_COUNT; ++i)
        {
            const double z = 1.0 - 2.0 * (static_cast<double>(i) + 0.5) / static_cast<double>(SAMPLE_COUNT);
            const double ring = std::sqrt(1.0 - z * z);
            const double angle = goldenAngle * static_cast<double>(i);
            points.emplace_back(RADIUS * ring * std::cos(angle), RADIUS * ring * std::sin(angle), RADIUS * z);
            geometryIds.push_back({surfaceId});
        }
    }

    static void SetUpTestSuite()
    {
        converter_ = std::make_unique<Readers::TopoDS_ShapeConverter>(BRepPrimAPI_MakeSphere(RADIUS).Shape());
        const auto& topology = converter_->getTopology();
        const std::string surfaceId = topology.getAllSurfaceIds().front();

        std::vector<Point3D> points;
        std::vector<std::vector<std::string>> geometryIds;
        sampleSphere(surfaceId, points, geometryIds);

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
    std::map<std::pair<NodeId, NodeId>, int> edgeFaceCount;
    std::set<NodeId> vertices;
    for (const auto& [face, surfaceId] : faces_)
    {
        const auto& n = face.nodeIds;
        for (const auto& [a, b] : {std::make_pair(n[0], n[1]), std::make_pair(n[0], n[2]), std::make_pair(n[1], n[2])})
            ++edgeFaceCount[{std::min(a, b), std::max(a, b)}];
        vertices.insert(n.begin(), n.end());
    }
    for (const auto& [edge, count] : edgeFaceCount)
        EXPECT_EQ(count, 2) << "edge " << edge.first.value() << "-" << edge.second.value();
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
    const NodeId onTop = mutator.addBoundaryNode(Point3D(0.0, 0.0, 1.0), {"top"});
    const NodeId onSide = mutator.addBoundaryNode(Point3D(1.0, 0.0, 0.0), {"side"});
    const NodeId onCurve = mutator.addBoundaryNode(Point3D(0.0, 1.0, 0.0), {"crease"});
    const NodeId alsoOnTop = mutator.addBoundaryNode(Point3D(1.0, 1.0, 1.0), {"top"});

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

namespace
{

class ScopedEnvironmentOverride
{
public:
    ScopedEnvironmentOverride(const std::string& name, const std::string& value) :
        name_(name)
    {
        auto previous = std::getenv(name_.c_str());
        hadPreviousValue_ = previous != nullptr;
        if (hadPreviousValue_) previousValue_ = previous;
        setenv(name_.c_str(), value.c_str(), 1);
    }

    ~ScopedEnvironmentOverride()
    {
        if (hadPreviousValue_)
            setenv(name_.c_str(), previousValue_.c_str(), 1);
        else
            unsetenv(name_.c_str());
    }

private:
    std::string name_;
    bool hadPreviousValue_;
    std::string previousValue_;
};

size_t countLines(const std::filesystem::path& path)
{
    std::ifstream stream(path);
    size_t count = 0;
    std::string line;
    while (std::getline(stream, line))
        ++count;
    return count;
}

} // namespace

TEST_F(SurfaceDelaunayRefinerTest, WritesSeededInsertionAndRefinedSnapshots)
{
    auto directory = std::filesystem::temp_directory_path() / "openloom_surface_refiner_snapshots";
    std::filesystem::remove_all(directory);
    std::filesystem::create_directories(directory);
    ScopedEnvironmentOverride flag("EXPORT_MESH_EACH_ITERATION", "1");
    ScopedEnvironmentOverride stride("EXPORT_MESH_EVERY_N", "50");
    ScopedEnvironmentOverride output("EXPORT_MESH_DIRECTORY", directory.string());
    MeshSnapshotWriter::restartSequence();

    const auto& topology = converter_->getTopology();
    std::vector<Point3D> points;
    std::vector<std::vector<std::string>> geometryIds;
    sampleSphere(topology.getAllSurfaceIds().front(), points, geometryIds);
    MeshingContext3D context(converter_->getGeometryCollection(), topology);
    Delaunay3D::triangulate(context.getOperations(), points, geometryIds);

    SurfaceMesh3DQualitySettings settings;
    settings.minAngleDegrees = MINIMUM_ANGLE_DEGREES;
    settings.chordDeviationTolerance = 0.05;
    settings.maxRefinementIterations = 5000;
    SurfaceDelaunayRefiner refiner(context, topology, settings, MINIMUM_EDGE_LENGTH);
    refiner.refine();
    refiner.writeSnapshot("refined");

    std::vector<std::string> labels;
    for (auto& entry : std::filesystem::directory_iterator(directory))
    {
        auto name = entry.path().filename().string();
        if (name.ends_with(".vtu")) labels.push_back(name.substr(0, name.size() - 4));
    }
    std::sort(labels.begin(), labels.end());

    ASSERT_GE(labels.size(), 4u);
    EXPECT_EQ(labels[0], "rcdt_000000_seeded");
    EXPECT_EQ(labels[1], "rcdt_000001_balls");
    EXPECT_NE(labels[2].find("_insertion"), std::string::npos);
    EXPECT_NE(labels.back().find("_refined"), std::string::npos);

    // One header row plus one row per restricted facet.
    auto cellsLines = countLines(directory / (labels.back() + ".cells.tsv"));
    EXPECT_EQ(cellsLines - 1, refiner.getRestrictedFaces().size());

    std::ifstream refinedCells(directory / (labels.back() + ".cells.tsv"));
    std::string header;
    std::getline(refinedCells, header);
    EXPECT_EQ(header, "kind\tid\tnodes\tSurfaceID\tBadnessCriterion\tBadnessValue\tDropped");

    // The sphere has no curves, so the protecting balls are empty: header only.
    EXPECT_EQ(countLines(directory / (labels[1] + ".nodes.tsv")), 1u);

    std::filesystem::remove_all(directory);
}
