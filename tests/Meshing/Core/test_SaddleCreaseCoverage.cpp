#include "SaddleShape.h"

#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Connectivity/FaceKey.h"
#include "Meshing/Core/3D/RCDT/RestrictedTriangulation.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"
#include "Meshing/Data/3D/TetrahedralElement.h"
#include "Meshing/Data/Base/MeshConnectivity.h"
#include "Meshing/Data/CurveSegmentManager.h"
#include "Readers/OpenCascade/TopoDS_ShapeConverter.h"
#include "Topology/Edge3D.h"
#include "Topology/Topology3D.h"

#include <TopoDS_Shape.hxx>

#include <algorithm>
#include <array>
#include <map>
#include <memory>
#include <string>
#include <set>
#include <vector>

#include <gtest/gtest.h>

using namespace Meshing;

// ============================================================================
// SaddleCreaseCoverageTest
//
// The saddle loses the tips of both horns, and nothing reports it.
//
// At x = +/- 2 the top face meets a side face along a parabolic crease arc.
// The four samples nearest each crest end up in NO restricted face: at stock
// density, node ids 19-22 on the x = +2 arc and 31-34 on x = -2. The surface
// mesh chords straight across the top of each ear, cutting the horn off.
//
// It goes unreported because RestrictedFaceAudit::findNonManifoldEdges()
// checks EDGE coverage -- how many faces each edge of the restricted set
// carries against what the CAD topology calls for. A node used by no face
// contributes no edge, so an orphaned node is structurally invisible to it.
// That is how SaddleSurfaceMesh reports "0 holes" while visibly missing its
// horn tips. Recorded from a picture in the OPE-184 notes ("4 orphaned nodes
// each") and unattributed until now.
//
// ## Why five nodes and not a meshing run
//
// The coordinates below are lifted verbatim from a real SaddleSurfaceMesh run
// (EXPORT_PHASE_DIAGNOSTICS=1, stock density), so this is the geometry that
// actually fails -- but meshing the saddle to reach it costs 90 seconds, and
// the question is about one classification call.
//
// ## What the extraction showed, which is the finding
//
// Node 20 has 21 incident faces in the tetrahedralization. Every one is either
// a chord along the crease -- like the face under test, three CONSECUTIVE arc
// samples -- or a triangle spanning the whole model to the opposite crease at
// x = -2. There are no nearby top-surface or side-surface points to connect
// to, because refinement never inserted any at the crest.
//
// And a face on three consecutive samples cannot survive either: its
// first-to-third edge skips the middle sample, so it is a same-curve chord
// edge by construction. Every candidate at the crest is thus either a chord or
// model-spanning.
//
// So the orphan is not a misclassification. The classifier is never offered a
// usable face. That places the cause upstream, in refinement being unable to
// insert near a protected crease (OPE-176 measured 64 of 72 blocked big
// triangles tracing to encroachesProtectingBall), rather than in the
// restriction oracle OPE-186 replaces.
//
// The assertion is therefore deliberately weak: that the best candidate the
// tetrahedralization actually offers survives to the output. Satisfying it
// would un-orphan nodes 19 and 20 -- though a sliver of three consecutive
// crease samples is a poor triangle, and the real fix is for better candidates
// to exist at all.
// ============================================================================

namespace
{

// Three consecutive samples on the x = +2 crease arc, and the apexes of the
// two tetrahedra sharing them -- one reaching outside the model, one reaching
// across to the far crease. Verbatim from the run.
constexpr double CREST_0[3] = {2.000000, -0.672000, 3.548416};
constexpr double CREST_1[3] = {2.000000, -0.296000, 3.912384};
constexpr double CREST_2[3] = {2.000000, -0.068000, 3.995376};
constexpr double APEX_OUTSIDE[3] = {4.401612, -4.247611, -3.937440};
constexpr double APEX_FAR_SIDE[3] = {-2.000000, -0.068000, 3.995376};

Point3D pointOf(const double (&coordinates)[3])
{
    return Point3D(coordinates[0], coordinates[1], coordinates[2]);
}

class SaddleCreaseCoverageTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        shape_ = TestSupport::buildSaddleSolid();
        converter_ = std::make_unique<Readers::TopoDS_ShapeConverter>(shape_);
    }

    static void TearDownTestSuite() { converter_.reset(); }

    /// Every surface the point lies on, within its trimmed patch.
    static std::vector<std::string> surfacesAt(const Point3D& point)
    {
        std::vector<std::string> ids;
        for (const auto& surfaceId : converter_->getTopology().getAllSurfaceIds())
        {
            const Geometry3D::ISurface3D* surface = converter_->getGeometryCollection().getSurface(surfaceId);
            if (!surface || surface->getGap(point) > 1e-6)
                continue;
            const auto uv = surface->projectPointToUnderlyingSurface(point);
            if (uv.has_value() && surface->isUVWithinTrimmedBoundary(uv->x(), uv->y()))
                ids.push_back(surfaceId);
        }
        return ids;
    }

    /// The surfaces all three crest samples share -- the two the crease joins.
    static std::vector<std::string> sharedSurfacesOfCrest()
    {
        const auto first = surfacesAt(pointOf(CREST_0));
        const auto second = surfacesAt(pointOf(CREST_1));
        const auto third = surfacesAt(pointOf(CREST_2));

        std::vector<std::string> shared;
        for (const auto& id : first)
        {
            if (std::find(second.begin(), second.end(), id) != second.end() &&
                std::find(third.begin(), third.end(), id) != third.end())
                shared.push_back(id);
        }
        return shared;
    }

    /// The model curve joining those surfaces -- the x = +2 crease, which is
    /// what BoundaryDiscretizer3D tags a crease sample with.
    static std::string creaseCurveId()
    {
        const auto shared = sharedSurfacesOfCrest();
        for (const auto& edgeId : converter_->getTopology().getAllEdgeIds())
        {
            const auto& adjacent = converter_->getTopology().getEdge(edgeId).getAdjacentSurfaceIds();
            if (adjacent.size() != shared.size())
                continue;
            const bool matches =
                std::all_of(shared.begin(),
                            shared.end(),
                            [&adjacent](const std::string& surfaceId)
                            { return std::find(adjacent.begin(), adjacent.end(), surfaceId) != adjacent.end(); });
            if (matches)
                return edgeId;
        }
        return {};
    }

    /// The model curve adjacent to exactly this set of surfaces.
    static std::string curveIdJoining(const std::vector<std::string>& surfaces)
    {
        for (const auto& edgeId : converter_->getTopology().getAllEdgeIds())
        {
            const auto& adjacent = converter_->getTopology().getEdge(edgeId).getAdjacentSurfaceIds();
            if (adjacent.size() != surfaces.size())
                continue;
            const bool matches =
                std::all_of(surfaces.begin(),
                            surfaces.end(),
                            [&adjacent](const std::string& surfaceId)
                            { return std::find(adjacent.begin(), adjacent.end(), surfaceId) != adjacent.end(); });
            if (matches)
                return edgeId;
        }
        return {};
    }

    /// One segment between each pair of crease nodes adjacent along their
    /// shared curve, ordered by arc position -- what CurveSegmentBuilder
    /// produces from the discretization.
    static CurveSegmentManager buildCreaseSegments(const MeshData3D& meshData, const std::vector<size_t>& nodeIds)
    {
        const auto allEdgeIds = converter_->getTopology().getAllEdgeIds();
        const std::set<std::string> curveIds(allEdgeIds.begin(), allEdgeIds.end());

        std::map<std::string, std::vector<size_t>> byCurve;
        for (const size_t nodeId : nodeIds)
            for (const auto& id : meshData.getGeometryIds(nodeId))
                if (curveIds.count(id))
                    byCurve[id].push_back(nodeId);

        CurveSegmentManager segments;
        for (auto& [curveId, ids] : byCurve)
        {
            std::sort(ids.begin(),
                      ids.end(),
                      [&meshData](size_t a, size_t b)
                      {
                          const Point3D& pa = meshData.getNode(a)->getCoordinates();
                          const Point3D& pb = meshData.getNode(b)->getCoordinates();
                          return pa.y() != pb.y() ? pa.y() < pb.y() : pa.x() < pb.x();
                      });
            for (size_t i = 0; i + 1 < ids.size(); ++i)
                segments.addSegment(CurveSegment{ids[i],
                                                 ids[i + 1],
                                                 curveId,
                                                 static_cast<double>(i),
                                                 static_cast<double>(i + 1)});
        }
        return segments;
    }

    static TopoDS_Shape shape_;
    static std::unique_ptr<Readers::TopoDS_ShapeConverter> converter_;
};

TopoDS_Shape SaddleCreaseCoverageTest::shape_;
std::unique_ptr<Readers::TopoDS_ShapeConverter> SaddleCreaseCoverageTest::converter_;

} // namespace

// The three crest samples lie on a common crease, so the face is a legitimate
// candidate rather than one the classifier may reject out of hand.
TEST_F(SaddleCreaseCoverageTest, TheCrestSamplesLieOnACommonCrease)
{
    EXPECT_GE(sharedSurfacesOfCrest().size(), 2u)
        << "the crest samples should sit on the two surfaces the crease joins";
    EXPECT_FALSE(creaseCurveId().empty()) << "no model curve joins those surfaces";
}

#include "SaddleCrestPatch.inc"

// THE DEFECT, on the real neighbourhood. Two tetrahedra are not enough: the
// crest face is only droppable once its non-chord edges carry other faces, so
// the orphaning is a property of the surrounding patch, not of the face. This
// is every tetrahedron incident to the crest samples -- 83 of them over 31
// nodes, against 7481 in the full mesh.
TEST_F(SaddleCreaseCoverageTest, TheCrestNodesAreUsedBySomeTriangle)
{
    const std::string curveId = creaseCurveId();
    ASSERT_FALSE(curveId.empty());

    MeshData3D meshData;
    std::vector<size_t> nodeIds;
    {
        MeshMutator3D mutator(meshData);
        for (int i = 0; i < CREST_PATCH_NODE_COUNT; ++i)
        {
            const Point3D point(CREST_PATCH_NODES[i][0], CREST_PATCH_NODES[i][1], CREST_PATCH_NODES[i][2]);
            // Tags derived from position rather than baked in: a point on two
            // surfaces is a crease sample and carries the curve id, a point on
            // one carries that surface, anything else is an ordinary node.
            const auto surfaces = surfacesAt(point);
            std::vector<std::string> tags;
            if (surfaces.size() >= 2)
                tags.push_back(curveIdJoining(surfaces));
            else if (surfaces.size() == 1)
                tags.push_back(surfaces.front());
            nodeIds.push_back(tags.empty() ? mutator.addNode(point) : mutator.addBoundaryNode(point, tags));
        }
        for (int i = 0; i < CREST_PATCH_TET_COUNT; ++i)
        {
            mutator.addElement(std::make_unique<TetrahedralElement>(
                std::array<size_t, 4>{nodeIds[CREST_PATCH_TETS[i][0]], nodeIds[CREST_PATCH_TETS[i][1]],
                                      nodeIds[CREST_PATCH_TETS[i][2]], nodeIds[CREST_PATCH_TETS[i][3]]}));
        }
        mutator.setCurveSegmentManager(buildCreaseSegments(meshData, nodeIds));
    }

    const MeshConnectivity connectivity(meshData);
    RestrictedTriangulation restrictedTriangulation;
    restrictedTriangulation.buildFrom(meshData, connectivity, converter_->getGeometryCollection(),
                                      converter_->getTopology(), 0.0523, SurfaceMesh3DQualitySettings{});
    restrictedTriangulation.removeDefectiveFaces(meshData);

    std::set<size_t> used;
    for (const auto& [face, surfaceId] : restrictedTriangulation.getRestrictedFaces())
        for (const size_t nodeId : face.nodeIds)
            used.insert(nodeId);

    for (const int orphan : CREST_PATCH_ORPHANS)
    {
        const size_t nodeId = nodeIds[orphan];
        EXPECT_TRUE(used.count(nodeId))
            << "crest node at (" << CREST_PATCH_NODES[orphan][0] << ", " << CREST_PATCH_NODES[orphan][1] << ", "
            << CREST_PATCH_NODES[orphan][2] << ") appears in no restricted face";
    }
}

// THE DEFECT. The best candidate the tetrahedralization offers at the crest
// does not survive to the output, so nodes 19 and 20 end up in no triangle.
TEST_F(SaddleCreaseCoverageTest, TheBestCandidateFaceAtTheCrestSurvives)
{
    const std::string curveId = creaseCurveId();
    ASSERT_FALSE(curveId.empty());

    MeshData3D meshData;
    std::array<size_t, 3> faceNodes{};
    {
        MeshMutator3D mutator(meshData);

        // Tagged with the CURVE, as the discretizer tags a crease sample --
        // not with the surfaces it happens to lie on. The difference is
        // load-bearing: isSameCurveChordEdge() and the protected-edge route
        // both key off the curve id.
        faceNodes[0] = mutator.addBoundaryNode(pointOf(CREST_0), {curveId});
        faceNodes[1] = mutator.addBoundaryNode(pointOf(CREST_1), {curveId});
        faceNodes[2] = mutator.addBoundaryNode(pointOf(CREST_2), {curveId});

        const size_t apexOutside = mutator.addNode(pointOf(APEX_OUTSIDE));
        const size_t apexFarSide = mutator.addBoundaryNode(pointOf(APEX_FAR_SIDE), {curveId});

        mutator.addElement(std::make_unique<TetrahedralElement>(
            std::array<size_t, 4>{faceNodes[0], faceNodes[1], faceNodes[2], apexOutside}));
        mutator.addElement(std::make_unique<TetrahedralElement>(
            std::array<size_t, 4>{faceNodes[0], faceNodes[1], faceNodes[2], apexFarSide}));

        // Consecutive crest samples are chain-adjacent along the crease, which
        // is exactly what makes the first-to-third edge a CHORD rather than a
        // third protected edge.
        CurveSegmentManager segments;
        segments.addSegment(CurveSegment{faceNodes[0], faceNodes[1], curveId, 0.0, 1.0});
        segments.addSegment(CurveSegment{faceNodes[1], faceNodes[2], curveId, 1.0, 2.0});
        mutator.setCurveSegmentManager(std::move(segments));
    }

    const MeshConnectivity connectivity(meshData);
    RestrictedTriangulation restrictedTriangulation;
    restrictedTriangulation.buildFrom(meshData,
                                      connectivity,
                                      converter_->getGeometryCollection(),
                                      converter_->getTopology(),
                                      0.0523,
                                      SurfaceMesh3DQualitySettings{});

    const FaceKey crestFace(faceNodes);
    EXPECT_TRUE(restrictedTriangulation.getRestrictedFaces().count(crestFace))
        << "the face on three consecutive crease samples was not classified as surface";

    restrictedTriangulation.removeDefectiveFaces(meshData);

    EXPECT_TRUE(restrictedTriangulation.getRestrictedFaces().count(crestFace))
        << "the only candidate the tetrahedralization offers at the crest is dropped as a "
           "same-curve chord face, which is why nodes 19 and 20 at the horn tip appear in "
           "no triangle at all";
}
