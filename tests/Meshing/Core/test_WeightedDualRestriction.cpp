#include "Meshing/Core/3D/RCDT/WeightedDualRestriction.h"

#include "Meshing/Connectivity/EdgeKey.h"
#include "Meshing/Connectivity/FaceKey.h"
#include "Meshing/Core/3D/General/MeshingContext3D.h"
#include "Meshing/Core/3D/Volume/Delaunay3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"
#include "Meshing/Data/Base/MeshConnectivity.h"
#include "Readers/OpenCascade/TopoDS_ShapeConverter.h"
#include "Topology/Topology3D.h"

#include <BRepPrimAPI_MakeSphere.hxx>

#include <cmath>
#include <map>
#include <memory>
#include <numbers>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>

using namespace Meshing;

// ============================================================================
// WeightedDualRestrictionTest
//
// The restricted Delaunay theorem, checked on the case it is proved for: a
// dense enough sample of a smooth closed surface restricts to a triangulation
// of that surface. A sphere of radius 3 sampled by 300 points spread evenly
// (spacing about 0.6 against a local feature size of 3) is well inside that
// regime, so the restricted facets must form a closed 2-manifold of genus 0,
// and every surface Delaunay ball centre must lie on the sphere.
//
// Nothing here is tuned: no gate, no shortcut, no inside/outside test. If this
// fails, the restriction test itself is wrong.
// ============================================================================

class WeightedDualRestrictionTest : public ::testing::Test
{
protected:
    static constexpr double RADIUS = 3.0;
    static constexpr size_t SAMPLE_COUNT = 300;
    static constexpr double MINIMUM_EDGE_LENGTH = 0.1;

    static void SetUpTestSuite()
    {
        converter_ = std::make_unique<Readers::TopoDS_ShapeConverter>(BRepPrimAPI_MakeSphere(RADIUS).Shape());
        const auto& geometry = converter_->getGeometryCollection();
        const auto& topology = converter_->getTopology();
        surfaceId_ = topology.getAllSurfaceIds().front();

        // Fibonacci sphere: evenly spread, no two points coincide.
        std::vector<Point3D> points;
        std::vector<std::vector<std::string>> geometryIds;
        const double goldenAngle = std::numbers::pi * (3.0 - std::sqrt(5.0));
        for (size_t i = 0; i < SAMPLE_COUNT; ++i)
        {
            const double z = 1.0 - 2.0 * (static_cast<double>(i) + 0.5) / static_cast<double>(SAMPLE_COUNT);
            const double ring = std::sqrt(1.0 - z * z);
            const double angle = goldenAngle * static_cast<double>(i);
            points.emplace_back(RADIUS * ring * std::cos(angle), RADIUS * ring * std::sin(angle), RADIUS * z);
            geometryIds.push_back({surfaceId_});
        }

        context_ = std::make_unique<MeshingContext3D>(geometry, topology);
        Delaunay3D::triangulate(context_->getOperations(), points, geometryIds);

        const auto& meshData = context_->getMeshData();
        const MeshConnectivity connectivity(meshData);
        const WeightedDualRestriction restriction(geometry, topology, MINIMUM_EDGE_LENGTH);

        std::set<FaceKey> seen;
        for (const auto& [elementId, element] : meshData.getElements())
        {
            const auto* tetrahedron = dynamic_cast<const TetrahedralElement*>(element.get());
            if (!tetrahedron)
                continue;
            for (const auto& faceNodes : tetrahedron->getFaces())
            {
                const FaceKey face(faceNodes);
                if (!seen.insert(face).second)
                    continue;
                if (const auto facet = restriction.restrict(face, meshData, connectivity))
                    facets_.emplace(face, *facet);
            }
        }
    }

    static void TearDownTestSuite()
    {
        facets_.clear();
        context_.reset();
        converter_.reset();
    }

    static std::unique_ptr<Readers::TopoDS_ShapeConverter> converter_;
    static std::unique_ptr<MeshingContext3D> context_;
    static std::string surfaceId_;
    static std::map<FaceKey, RestrictedFacet> facets_;
};

std::unique_ptr<Readers::TopoDS_ShapeConverter> WeightedDualRestrictionTest::converter_;
std::unique_ptr<MeshingContext3D> WeightedDualRestrictionTest::context_;
std::string WeightedDualRestrictionTest::surfaceId_;
std::map<FaceKey, RestrictedFacet> WeightedDualRestrictionTest::facets_;

TEST_F(WeightedDualRestrictionTest, RestrictedFacetsFormAClosedGenusZeroSurface)
{
    ASSERT_FALSE(facets_.empty());

    std::map<std::pair<size_t, size_t>, int> edgeFaceCount;
    std::set<size_t> vertices;
    for (const auto& [face, facet] : facets_)
    {
        const auto& n = face.nodeIds;
        for (const auto& [a, b] : {std::make_pair(n[0], n[1]), std::make_pair(n[0], n[2]), std::make_pair(n[1], n[2])})
            ++edgeFaceCount[{std::min(a, b), std::max(a, b)}];
        vertices.insert(n.begin(), n.end());
    }

    for (const auto& [edge, count] : edgeFaceCount)
        EXPECT_EQ(count, 2) << "edge " << edge.first << "-" << edge.second;

    const auto eulerCharacteristic = static_cast<long>(vertices.size()) - static_cast<long>(edgeFaceCount.size()) +
                                     static_cast<long>(facets_.size());
    EXPECT_EQ(eulerCharacteristic, 2);
    EXPECT_EQ(vertices.size(), SAMPLE_COUNT);
}

TEST_F(WeightedDualRestrictionTest, SurfaceCentersLieOnTheSphere)
{
    for (const auto& [face, facet] : facets_)
    {
        EXPECT_EQ(facet.surfaceId, surfaceId_);
        EXPECT_NEAR(facet.surfaceCenter.norm(), RADIUS, 1e-6);
    }
}
