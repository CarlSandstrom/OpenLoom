#include "Meshing/Core/3D/RCDT/RCDTRefiner.h"

#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/IEdge3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Core/3D/General/MeshOperations3D.h"
#include "Meshing/Core/3D/General/MeshQueries3D.h"
#include "Meshing/Core/3D/General/MeshingContext3D.h"
#include "Meshing/Core/3D/RCDT/CurveSegmentOperations.h"
#include "Meshing/Core/3D/RCDT/RestrictedTriangulation.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Meshing/Data/3D/Node3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"
#include "Meshing/Data/Base/MeshConnectivity.h"
#include "Meshing/Data/CurveSegmentManager.h"
#include "spdlog/spdlog.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace Meshing
{

namespace
{

constexpr size_t MAX_ITERATIONS = 50000;

} // namespace

RCDTRefiner::RCDTRefiner(MeshingContext3D& context,
                         RestrictedTriangulation& restrictedTriangulation,
                         const RCDTQualitySettings& settings) :
    context_(&context),
    restrictedTriangulation_(&restrictedTriangulation),
    settings_(settings)
{
}

void RCDTRefiner::refine()
{
    const auto& meshData = context_->getMeshData();
    spdlog::info("RCDTRefiner: starting refinement — {} nodes, {} segments",
                 meshData.getNodeCount(),
                 meshData.getCurveSegmentManager().size());

    size_t iteration = 0;
    while (iteration < MAX_ITERATIONS)
    {
        if (!refineStep())
            break;
        ++iteration;
    }

    if (iteration >= MAX_ITERATIONS)
        spdlog::warn("RCDTRefiner: reached iteration cap ({})", MAX_ITERATIONS);

    spdlog::info("RCDTRefiner: done after {} iterations — {} nodes",
                 iteration,
                 context_->getMeshData().getNodeCount());
}

bool RCDTRefiner::refineStep()
{
    const auto& meshData = context_->getMeshData();
    const auto& curveSegmentManager = meshData.getCurveSegmentManager();

    // ---- Priority 1: encroached curve segments ----

    const auto nodePositionMap = buildNodePositionMap();

    std::unordered_set<size_t> encroached;
    for (const auto& [nodeId, node] : meshData.getNodes())
    {
        for (const size_t segmentId :
             curveSegmentManager.findEncroached(node->getCoordinates(), nodePositionMap))
        {
            encroached.insert(segmentId);
        }
    }

    if (!encroached.empty())
        return splitSegment(*encroached.begin());

    // ---- Priority 2: bad restricted triangles ----

    const auto* geometry = context_->getGeometry();
    if (!geometry)
        return false;

    const auto badTriangles =
        restrictedTriangulation_->getBadTriangles(settings_, meshData, *geometry);

    if (badTriangles.empty())
        return false;

    const double diameter = computeMeshDiameter();

    for (const auto& bad : badTriangles)
    {
        if (unrefinableTriangles_.count(bad.face))
            continue;

        const Geometry3D::ISurface3D* surface = geometry->getSurface(bad.surfaceId);
        if (!surface)
        {
            unrefinableTriangles_.insert(bad.face);
            continue;
        }

        const auto projectedOpt =
            surfaceProjector_.projectToSurface(bad.circumcircleCenter, *surface);
        if (!projectedOpt)
        {
            unrefinableTriangles_.insert(bad.face);
            continue;
        }

        const Point3D& projected = *projectedOpt;

        // Proximity guard: skip if too close to any existing node.
        bool tooClose = false;
        for (const auto& [nodeId, node] : meshData.getNodes())
        {
            if ((projected - node->getCoordinates()).norm() < 1e-10 * diameter)
            {
                tooClose = true;
                break;
            }
        }
        if (tooClose)
        {
            unrefinableTriangles_.insert(bad.face);
            continue;
        }

        // Demotion: if the circumcenter would encroach a segment, split that segment instead.
        const auto encroachingIds = curveSegmentManager.findEncroached(projected, nodePositionMap);
        if (!encroachingIds.empty())
            return splitSegment(encroachingIds[0]);

        // Insert the projected circumcenter.
        insertAndUpdate(projected, {bad.surfaceId});
        unrefinableTriangles_.clear();
        return true;
    }

    return false;
}

size_t RCDTRefiner::insertAndUpdate(const Point3D& point,
                                    const std::vector<std::string>& geometryIds)
{
    auto& operations = context_->getOperations();
    const auto& meshData = context_->getMeshData();
    const auto* geometry = context_->getGeometry();

    const auto conflictingTets = operations.getQueries().findConflictingTetrahedra(point);
    const auto interiorFaces = computeCavityInteriorFaces(conflictingTets);

    const size_t newNodeId = operations.insertVertexBowyerWatson(point, geometryIds);

    const MeshConnectivity postConnectivity(meshData);
    restrictedTriangulation_->updateAfterInsertion(
        interiorFaces, newNodeId, meshData, postConnectivity, *geometry);

    return newNodeId;
}

bool RCDTRefiner::splitSegment(size_t segmentId)
{
    const auto& meshData = context_->getMeshData();
    const auto* geometry = context_->getGeometry();
    if (!geometry)
        return false;

    const CurveSegment& segment = meshData.getCurveSegmentManager().getSegment(segmentId);
    const Geometry3D::IEdge3D* edge = geometry->getEdge(segment.edgeId);
    if (!edge)
        return false;

    const Point3D splitPoint = computeSplitPoint(segment, *geometry);

    auto& operations = context_->getOperations();
    const auto conflictingTets = operations.getQueries().findConflictingTetrahedra(splitPoint);
    const auto interiorFaces = computeCavityInteriorFaces(conflictingTets);

    const size_t newNodeId = operations.insertVertexBowyerWatson(splitPoint, {segment.edgeId});

    const double tMid =
        edge->getParameterAtArcLengthFraction(segment.tStart, segment.tEnd, 0.5);
    context_->getMutator().splitCurveSegment(segmentId, newNodeId, tMid);

    const MeshConnectivity postConnectivity(meshData);
    restrictedTriangulation_->updateAfterInsertion(
        interiorFaces, newNodeId, meshData, postConnectivity, *geometry);

    unrefinableTriangles_.clear();
    return true;
}

double RCDTRefiner::computeMeshDiameter() const
{
    const auto& meshData = context_->getMeshData();
    if (meshData.getNodeCount() == 0)
        return 1.0;

    Point3D minPoint = Point3D::Constant(std::numeric_limits<double>::max());
    Point3D maxPoint = Point3D::Constant(std::numeric_limits<double>::lowest());

    for (const auto& [nodeId, node] : meshData.getNodes())
    {
        const Point3D& coordinates = node->getCoordinates();
        minPoint = minPoint.cwiseMin(coordinates);
        maxPoint = maxPoint.cwiseMax(coordinates);
    }

    return (maxPoint - minPoint).norm();
}

std::unordered_map<size_t, Point3D> RCDTRefiner::buildNodePositionMap() const
{
    std::unordered_map<size_t, Point3D> positionMap;
    for (const auto& [nodeId, node] : context_->getMeshData().getNodes())
        positionMap.emplace(nodeId, node->getCoordinates());
    return positionMap;
}

std::vector<FaceKey> RCDTRefiner::computeCavityInteriorFaces(
    const std::vector<size_t>& conflictingTets) const
{
    const auto& meshData = context_->getMeshData();

    std::unordered_map<FaceKey, size_t, FaceKeyHash> faceCount;
    for (const size_t tetId : conflictingTets)
    {
        const auto* element = meshData.getElement(tetId);
        const auto* tet = dynamic_cast<const TetrahedralElement*>(element);
        if (!tet)
            continue;

        for (const auto& faceArray : tet->getFaces())
            ++faceCount[FaceKey(faceArray)];
    }

    std::vector<FaceKey> interiorFaces;
    for (const auto& [face, count] : faceCount)
    {
        if (count == 2)
            interiorFaces.push_back(face);
    }

    return interiorFaces;
}

} // namespace Meshing
