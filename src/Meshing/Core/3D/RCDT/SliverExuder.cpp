#include "Meshing/Core/3D/RCDT/SliverExuder.h"

#include "Meshing/Connectivity/FaceKey.h"
#include "Meshing/Core/3D/General/ElementGeometry3D.h"
#include "Meshing/Core/3D/General/MeshOperations3D.h"
#include "Meshing/Core/3D/General/MeshingContext3D.h"
#include "Meshing/Core/3D/General/RegularConflictRegion.h"
#include "Meshing/Core/3D/General/RobustPredicates3D.h"
#include "Meshing/Core/3D/RCDT/AmbientTetrahedronClassifier.h"
#include "Meshing/Core/3D/RCDT/WeightedDualRestriction.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Meshing/Data/3D/TetrahedronElement.h"
#include "Meshing/Data/Base/MeshConnectivity.h"
#include "spdlog/spdlog.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <unordered_map>
#include <unordered_set>

namespace Meshing
{

namespace
{

// CGAL Slivers_exuder's sq_delta: a pumped weight stays below this fraction of
// the squared distance to the vertex's nearest neighbour.
constexpr double SQUARED_DELTA = 0.15;

// Weights tried per vertex, evenly spaced up to that limit.
constexpr int WEIGHT_STEPS = 16;

// Passes over the slivers; each starts from a fresh inside/outside labelling.
constexpr int MAXIMUM_PASSES = 10;

std::array<Point3D, 4> pointsOf(const std::array<NodeId, 4>& nodeIds, const MeshData3D& meshData);
std::array<NodeId, 4> nodeIdsOf(ElementId elementId, const MeshData3D& meshData);
bool isBoundingNode(NodeId nodeId, const MeshData3D& meshData);
double worstAngleOf(const std::vector<ElementId>& elements, const MeshData3D& meshData);
std::optional<double> worstAngleAfter(NodeId nodeId,
                                      const std::vector<ElementId>& cavity,
                                      const MeshData3D& meshData,
                                      const MeshQueries3D& queries);
bool destroysRestrictedFace(NodeId nodeId,
                            const std::vector<ElementId>& cavity,
                            const MeshData3D& meshData,
                            const RestrictedFaceMap& restrictedFaces);
std::unordered_set<FaceKey, FaceKeyHash> facesOf(const std::vector<ElementId>& elements, const MeshData3D& meshData);

} // namespace

SliverExuder::SliverExuder(MeshingContext3D& context,
                           const WeightedDualRestriction& restriction,
                           const RestrictedFaceMap& restrictedFaces,
                           double sliverAngleDegrees) :
    context_(&context),
    restriction_(&restriction),
    restrictedFaces_(&restrictedFaces),
    sliverAngleDegrees_(sliverAngleDegrees)
{
}

void SliverExuder::exude()
{
    if (!(sliverAngleDegrees_ > 0.0)) return;

    auto initial = findSlivers();
    std::size_t pumped = 0;
    for (int pass = 0; pass < MAXIMUM_PASSES; ++pass)
    {
        bool changed = false;
        for (auto [angle, elementId] : findSlivers())
        {
            if (!context_->getMeshData().getElement(elementId)) continue;
            for (auto nodeId : nodeIdsOf(elementId, context_->getMeshData()))
            {
                if (!pumpVertex(nodeId)) continue;
                ++pumped;
                changed = true;
                break;
            }
        }
        if (!changed) break;
    }

    auto remaining = findSlivers();
    spdlog::info("SliverExuder: {} vertices pumped; slivers below {} degrees {} -> {}, worst {:.3f} -> {:.3f} degrees",
                 pumped, sliverAngleDegrees_, initial.size(), remaining.size(),
                 initial.empty() ? sliverAngleDegrees_ : initial.front().first,
                 remaining.empty() ? sliverAngleDegrees_ : remaining.front().first);
}

/// The slivers inside the domain, worst first.
std::vector<std::pair<double, ElementId>> SliverExuder::findSlivers() const
{
    auto& meshData = context_->getMeshData();
    auto ambient = AmbientTetrahedronClassifier::classify(meshData, context_->getConnectivity(), *restrictedFaces_);
    std::vector<std::pair<double, ElementId>> slivers;
    for (auto& [elementId, element] : meshData.getElements())
    {
        if (!dynamic_cast<const TetrahedronElement*>(element.get()) || ambient.contains(elementId)) continue;
        auto angle = ElementGeometry3D::computeMinimumDihedralAngle(pointsOf(nodeIdsOf(elementId, meshData), meshData));
        if (angle < sliverAngleDegrees_) slivers.emplace_back(angle, elementId);
    }
    std::sort(slivers.begin(), slivers.end());
    return slivers;
}

/// Raises nodeId's weight to the candidate that leaves the best worst angle
/// around it and keeps every restriction. False if no candidate does.
bool SliverExuder::pumpVertex(NodeId nodeId)
{
    auto& meshData = context_->getMeshData();
    if (isBoundingNode(nodeId, meshData) || meshData.getNode(nodeId)->getWeight() != 0.0) return false;

    auto candidates = findCandidates(nodeId);
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& left, const Candidate& right)
              { return left.worstAngle > right.worstAngle; });
    for (auto& candidate : candidates)
        if (apply(nodeId, candidate)) return true;
    return false;
}

/// Every tried weight whose retriangulation is valid and improves the worst
/// angle among the tetrahedra it replaces. Read-only.
std::vector<SliverExuder::Candidate> SliverExuder::findCandidates(NodeId nodeId) const
{
    auto& meshData = context_->getMeshData();
    auto& connectivity = context_->getConnectivity();
    auto& point = meshData.getNode(nodeId)->getCoordinates();

    double nearestSquared = std::numeric_limits<double>::max();
    for (auto elementId : connectivity.getNodeElements(nodeId))
        for (auto otherId : nodeIdsOf(elementId, meshData))
            if (otherId != nodeId)
                nearestSquared = std::min(nearestSquared, (meshData.getNode(otherId)->getCoordinates() - point).squaredNorm());
    auto maximumWeight = SQUARED_DELTA * nearestSquared;

    std::vector<Candidate> candidates;
    for (int step = 1; step <= WEIGHT_STEPS; ++step)
    {
        auto weight = maximumWeight * step / WEIGHT_STEPS;
        auto cavity = RegularConflictRegion::findForWeightIncrease(nodeId, weight, meshData, connectivity);
        auto after = worstAngleAfter(nodeId, cavity, meshData, context_->getOperations().getQueries());
        if (!after) continue;
        if (*after > worstAngleOf(cavity, meshData) && !destroysRestrictedFace(nodeId, cavity, meshData, *restrictedFaces_))
            candidates.push_back({weight, *after});
    }
    return candidates;
}

/// Applies candidate, then undoes it unless the restriction is unchanged. The
/// cavity is found afresh: undoing an earlier candidate renumbers the
/// tetrahedra it restores.
bool SliverExuder::apply(NodeId nodeId, const Candidate& candidate)
{
    auto& meshData = context_->getMeshData();
    auto oldWeight = meshData.getNode(nodeId)->getWeight();
    auto cavity =
        RegularConflictRegion::findForWeightIncrease(nodeId, candidate.weight, meshData, context_->getConnectivity());
    auto oldFaces = facesOf(cavity, meshData);
    auto removed = context_->getOperations().raiseVertexWeight(nodeId, candidate.weight, cavity);
    if (keepsRestriction(nodeId, oldFaces)) return true;
    restore(nodeId, oldWeight, removed);
    return false;
}

/// Whether the weight change kept the restriction: no restricted face of the
/// replaced tetrahedra (oldFaces) is gone, and every face of the tetrahedra
/// around nodeId -- all the faces the change created or gave a new
/// neighbour -- is restricted exactly as before, to the same surface.
bool SliverExuder::keepsRestriction(NodeId nodeId, const std::unordered_set<FaceKey, FaceKeyHash>& oldFaces) const
{
    auto& meshData = context_->getMeshData();
    auto& connectivity = context_->getConnectivity();
    auto faces = facesOf(connectivity.getNodeElements(nodeId), meshData);
    for (auto& face : oldFaces)
        if (restrictedFaces_->contains(face) && !faces.contains(face)) return false;

    for (auto& face : faces)
    {
        auto restricted = restriction_->restrict(face, meshData, connectivity);
        auto before = restrictedFaces_->find(face);
        if (restricted.has_value() != (before != restrictedFaces_->end())) return false;
        if (restricted && restricted->surfaceId != before->second) return false;
    }
    return true;
}

void SliverExuder::restore(NodeId nodeId, double weight, const std::vector<std::array<NodeId, 4>>& removed)
{
    auto& mutator = context_->getMutator();
    auto star = context_->getConnectivity().getNodeElements(nodeId);
    for (auto elementId : star)
        mutator.removeElement(elementId);
    mutator.setNodeWeight(nodeId, weight);
    for (auto& nodeIds : removed)
        mutator.addElement(std::make_unique<TetrahedronElement>(nodeIds));
}

namespace
{

std::array<Point3D, 4> pointsOf(const std::array<NodeId, 4>& nodeIds, const MeshData3D& meshData)
{
    return {meshData.getNode(nodeIds[0])->getCoordinates(), meshData.getNode(nodeIds[1])->getCoordinates(),
            meshData.getNode(nodeIds[2])->getCoordinates(), meshData.getNode(nodeIds[3])->getCoordinates()};
}

std::array<NodeId, 4> nodeIdsOf(ElementId elementId, const MeshData3D& meshData)
{
    auto& nodeIds = dynamic_cast<const TetrahedronElement*>(meshData.getElement(elementId))->getNodeIds();
    return {nodeIds[0], nodeIds[1], nodeIds[2], nodeIds[3]};
}

bool isBoundingNode(NodeId nodeId, const MeshData3D& meshData)
{
    auto& bounding = meshData.getBoundingNodeIds();
    return bounding && std::find(bounding->begin(), bounding->end(), nodeId) != bounding->end();
}

double worstAngleOf(const std::vector<ElementId>& elements, const MeshData3D& meshData)
{
    double worst = 180.0;
    for (auto elementId : elements)
        worst = std::min(worst, ElementGeometry3D::computeMinimumDihedralAngle(pointsOf(nodeIdsOf(elementId, meshData), meshData)));
    return worst;
}

/// The worst angle of the tetrahedra nodeId would be fanned into over the
/// cavity's boundary. nullopt when the fan is not a valid retriangulation: a
/// cavity vertex missing from the boundary (raising the weight hides it), or
/// a boundary face flat with, or containing, nodeId.
std::optional<double> worstAngleAfter(NodeId nodeId,
                                      const std::vector<ElementId>& cavity,
                                      const MeshData3D& meshData,
                                      const MeshQueries3D& queries)
{
    auto boundary = queries.findCavityBoundary(cavity);
    std::unordered_set<NodeId> boundaryNodes;
    for (auto& face : boundary)
        boundaryNodes.insert(face.begin(), face.end());
    if (boundaryNodes.contains(nodeId)) return std::nullopt;
    for (auto elementId : cavity)
        for (auto otherId : nodeIdsOf(elementId, meshData))
            if (otherId != nodeId && !boundaryNodes.contains(otherId)) return std::nullopt;

    auto& apex = meshData.getNode(nodeId)->getCoordinates();
    double worst = 180.0;
    for (auto& face : boundary)
    {
        auto points = pointsOf({face[0], face[1], face[2], nodeId}, meshData);
        if (RobustPredicates3D::orientationSign(points[0], points[1], points[2], apex) == 0) return std::nullopt;
        worst = std::min(worst, ElementGeometry3D::computeMinimumDihedralAngle(points));
    }
    return worst;
}

/// Whether a restricted face lies inside the cavity where the weight change
/// is sure to destroy it. A face through nodeId may come back in the fan, so
/// SliverExuder::keepsRestriction() decides those.
bool destroysRestrictedFace(NodeId nodeId,
                            const std::vector<ElementId>& cavity,
                            const MeshData3D& meshData,
                            const RestrictedFaceMap& restrictedFaces)
{
    std::unordered_map<FaceKey, int, FaceKeyHash> count;
    for (auto elementId : cavity)
        for (auto& faceNodes : dynamic_cast<const TetrahedronElement*>(meshData.getElement(elementId))->getFaces())
            ++count[FaceKey(faceNodes)];
    for (auto& [face, occurrences] : count)
        if (occurrences == 2 && std::find(face.nodeIds.begin(), face.nodeIds.end(), nodeId) == face.nodeIds.end() &&
            restrictedFaces.contains(face)) return true;
    return false;
}

std::unordered_set<FaceKey, FaceKeyHash> facesOf(const std::vector<ElementId>& elements, const MeshData3D& meshData)
{
    std::unordered_set<FaceKey, FaceKeyHash> faces;
    for (auto elementId : elements)
        for (auto& faceNodes : dynamic_cast<const TetrahedronElement*>(meshData.getElement(elementId))->getFaces())
            faces.insert(FaceKey(faceNodes));
    return faces;
}

} // namespace

} // namespace Meshing
