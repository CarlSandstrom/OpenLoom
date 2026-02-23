#include "ShewchukRefiner2D.h"
#include "ElementGeometry2D.h"
#include "ElementQuality2D.h"
#include "Geometry/2D/Base/IEdge2D.h"
#include "MeshDebugUtils2D.h"
#include "MeshOperations2D.h"
#include "MeshQueries2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/MeshMutator2D.h"
#include "Meshing/Data/2D/Node2D.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/Base/MeshConnectivity.h"
#include "Topology2D/Topology2D.h"
#include "Utils/MeshLogger.h"
#include "spdlog/spdlog.h"
#include <algorithm>

namespace Meshing
{

ShewchukRefiner2D::ShewchukRefiner2D(MeshingContext2D& context,
                                     const IQualityController2D& qualityController) :
    context_(&context),
    qualityController_(&qualityController)
{
}

ShewchukRefiner2D::~ShewchukRefiner2D() = default;

void ShewchukRefiner2D::setOnBoundarySplit(BoundarySplitCallback callback)
{
    onBoundarySplit_ = std::move(callback);
}

void ShewchukRefiner2D::refine()
{
    spdlog::info("ShewchukRefiner2D: Starting mesh refinement");

    size_t iterationCount = 0;
    size_t consecutiveNoProgress = 0;
    const size_t maxIterations = 10000;         // Safety limit to prevent infinite loops
    const size_t maxConsecutiveNoProgress = 10; // Exit if no progress for this many iterations

    exportAndVerifyMesh(context_->getMeshData(), "ShewchukRefiner2D", exportCounter_);
    while (true)
    {
        // Check mesh quality
        MeshData3D meshData3D(context_->getMeshData());
        MeshConnectivity connectivity(meshData3D);

        if (qualityController_->isMeshAcceptable(context_->getMeshData(), connectivity))
        {
            break;
        }

        if (iterationCount >= maxIterations)
        {
            spdlog::warn("ShewchukRefiner2D: Reached maximum iteration limit");
            break;
        }

        bool refined = refineStep();
        if (!refined)
        {
            consecutiveNoProgress++;
            spdlog::debug("ShewchukRefiner2D: No refinement in iteration {} (consecutive: {})",
                          iterationCount, consecutiveNoProgress);

            if (consecutiveNoProgress >= maxConsecutiveNoProgress)
            {
                spdlog::warn("ShewchukRefiner2D: Unable to make progress after {} attempts. "
                             "Mesh may contain triangles too small to refine or quality goals may be unachievable.",
                             maxConsecutiveNoProgress);
                break;
            }
        }
        else
        {
            consecutiveNoProgress = 0; // Reset counter on successful refinement

            // Periodically re-classify triangles to remove any that ended up in holes
            // This prevents accumulation of invalid triangles during refinement
            if (iterationCount % 10 == 0)
            {
                spdlog::debug("ShewchukRefiner2D: Periodic triangle re-classification at iteration {}", iterationCount);
                auto removedIds = context_->getOperations().classifyAndRemoveExteriorTriangles();

                // Clean up unrefinableTriangles_ - remove IDs of triangles that were removed
                for (size_t id : removedIds)
                {
                    unrefinableTriangles_.erase(id);
                }
            }
        }

        ++iterationCount;
        exportAndVerifyMesh(context_->getMeshData(), "ShewchukRefiner2D", exportCounter_);
    }

    spdlog::info("ShewchukRefiner2D: Refinement complete after {} iterations", iterationCount);

    // Re-classify triangles after refinement to remove any that ended up in holes
    // Refinement can create triangles that cross constraint boundaries
    spdlog::info("ShewchukRefiner2D: Re-classifying triangles to remove any in holes");
    auto removedIds = context_->getOperations().classifyAndRemoveExteriorTriangles();

    // Clean up unrefinableTriangles_ - remove IDs of triangles that were removed
    for (size_t id : removedIds)
    {
        unrefinableTriangles_.erase(id);
    }
}

bool ShewchukRefiner2D::refineStep()
{
    // Priority 1: Handle encroached segments first
    auto encroachedSegments = context_->getOperations().getQueries().findEncroachedSegments();

    if (!encroachedSegments.empty())
    {
        spdlog::debug("ShewchukRefiner2D: Found {} encroached segments", encroachedSegments.size());
        handleEncroachedSegment(encroachedSegments[0]);
        return true;
    }

    // Priority 2: Refine worst quality triangle
    ElementQuality2D quality(context_->getMeshData());
    auto worstTriangles = quality.getTrianglesSortedByQuality();

    // Try to refine triangles in order of worst quality
    // Skip any where circumcenter computation fails or are known to be unrefinable
    for (size_t triangleId : worstTriangles)
    {
        // exportAndVerifyMesh disabled during refinement - flood fill at end will clean up
        const auto* element = context_->getMeshData().getElement(triangleId);
        const auto* triangle = dynamic_cast<const TriangleElement*>(element);

        if (triangle == nullptr)
            continue;

        // Skip triangles we've already determined can't be refined
        if (unrefinableTriangles_.contains(triangleId))
            continue;

        // Check if this triangle needs refinement
        if (!qualityController_->isTriangleAcceptable(*triangle))
        {
            spdlog::debug("ShewchukRefiner2D: Attempting to refine triangle {}", triangleId);

            // Try to refine this triangle
            if (handlePoorQualityTriangle(triangleId))
            {
                return true; // Successfully refined
            }
            // If refinement failed, mark as unrefinable (likely circumcenter in hole)
            // This prevents infinite loops trying to refine the same triangle
            unrefinableTriangles_.insert(triangleId);
        }
    }

    return false;
}

void ShewchukRefiner2D::handleEncroachedSegment(const ConstrainedSegment2D& segment)
{
    spdlog::debug("ShewchukRefiner2D: Splitting encroached segment ({}, {})",
                  segment.nodeId1, segment.nodeId2);

    // Find the common geometry ID (edge ID) between the two segment nodes
    auto commonGeometryId = context_->getOperations().getQueries().findCommonGeometryId(
        segment.nodeId1, segment.nodeId2);

    if (!commonGeometryId.has_value())
    {
        spdlog::error("ShewchukRefiner2D: Cannot split segment - no common geometry ID found between nodes {} and {}",
                      segment.nodeId1, segment.nodeId2);
        return;
    }

    // Get the parent edge geometry
    const auto* edge = context_->getGeometry().getEdge(commonGeometryId.value());
    if (edge == nullptr)
    {
        spdlog::error("ShewchukRefiner2D: Cannot find edge geometry for ID '{}'", commonGeometryId.value());
        return;
    }

    // Split the segment (also updates constrained segments in MeshData2D)
    auto newNodeId = context_->getOperations().splitConstrainedSegment(segment, *edge);

    if (!newNodeId.has_value())
    {
        spdlog::error("ShewchukRefiner2D: Failed to split segment ({}, {})", segment.nodeId1, segment.nodeId2);
        return;
    }

    spdlog::debug("ShewchukRefiner2D: Successfully split segment ({}, {}) at node {}",
                  segment.nodeId1, segment.nodeId2, newNodeId.value());

    if (onBoundarySplit_)
    {
        onBoundarySplit_(segment.nodeId1, segment.nodeId2, newNodeId.value());
    }
}

bool ShewchukRefiner2D::handlePoorQualityTriangle(size_t triangleId)
{
    const auto* element = context_->getMeshData().getElement(triangleId);
    const auto* triangle = dynamic_cast<const TriangleElement*>(element);

    if (triangle == nullptr)
    {
        spdlog::error("ShewchukRefiner2D: Triangle {} not found", triangleId);
        return false;
    }

    ElementGeometry2D geometry(context_->getMeshData());
    ElementQuality2D quality(context_->getMeshData());

    // Check if triangle is too small to refine reliably
    // Attempting to refine very small triangles leads to numerical issues
    double area = geometry.computeArea(*triangle);
    double shortestEdge = quality.computeShortestEdgeLength(*triangle);

    const double MIN_REFINABLE_AREA = 1e-12; // Minimum area threshold
    const double MIN_REFINABLE_EDGE = 1e-8;  // Minimum edge length threshold

    if (area < MIN_REFINABLE_AREA || shortestEdge < MIN_REFINABLE_EDGE)
    {
        spdlog::debug("ShewchukRefiner2D: Triangle {} too small to refine (area={:.2e}, shortest edge={:.2e}), skipping",
                      triangleId, area, shortestEdge);
        return false;
    }

    // Compute circumcenter
    auto circumcenterOpt = geometry.computeCircumcenter(*triangle);

    if (!circumcenterOpt.has_value())
    {
        spdlog::debug("ShewchukRefiner2D: Cannot compute circumcenter for triangle {} (likely degenerate), skipping", triangleId);
        return false;
    }

    Point2D circumcenter = circumcenterOpt.value();

    // Check if circumcenter is inside the domain (not in a hole)
    if (!context_->getOperations().getQueries().isPointInsideDomain(circumcenter))
    {
        spdlog::debug("ShewchukRefiner2D: Circumcenter at ({:.2f}, {:.2f}) is outside domain or in hole, skipping",
                      circumcenter.x(), circumcenter.y());
        return false;
    }

    // Check if circumcenter coincides with an existing mesh node.
    // This happens when the circumcenter lands at a circle center where a
    // node was already inserted. Bowyer-Watson cannot handle duplicate points.
    for (const auto& [nodeId, node] : context_->getMeshData().getNodes())
    {
        double dist = (node->getCoordinates() - circumcenter).norm();
        if (dist < MIN_REFINABLE_EDGE)
        {
            spdlog::debug("ShewchukRefiner2D: Circumcenter at ({:.2f}, {:.2f}) coincides with "
                          "existing node {}, skipping",
                          circumcenter.x(), circumcenter.y(), nodeId);
            return false;
        }
    }

    // Check if circumcenter would encroach any segments (with visibility check).
    // Per Ruppert: if the circumcenter would encroach a visible subsegment, reject
    // the circumcenter and split the encroached subsegment instead.
    auto encroachedByCircumcenter = context_->getOperations().getQueries().findSegmentsEncroachedByPoint(circumcenter);

    if (!encroachedByCircumcenter.empty())
    {
        spdlog::debug("ShewchukRefiner2D: Circumcenter would encroach {} segments, splitting them first",
                      encroachedByCircumcenter.size());

        handleEncroachedSegment(encroachedByCircumcenter[0]);
        return true;
    }

    // Safe to insert circumcenter
    spdlog::debug("ShewchukRefiner2D: Inserting circumcenter at ({:.2f}, {:.2f})",
                  circumcenter.x(), circumcenter.y());

    try
    {
        context_->getOperations().insertVertexBowyerWatson(circumcenter);

        // If the target triangle still exists after insertion, the circumcenter was
        // on the other side of a constrained edge and the insertion didn't help.
        // Mark it unrefinable to avoid an infinite loop.
        if (context_->getMeshData().getElement(triangleId) != nullptr)
        {
            spdlog::debug("ShewchukRefiner2D: Triangle {} survived circumcenter insertion, marking unrefinable",
                          triangleId);
            unrefinableTriangles_.insert(triangleId);
        }

        return true; // Successfully inserted circumcenter
    }
    catch (const std::exception& e)
    {
        // Circumcenter insertion failed (likely because it's inside a hole or creates invalid triangles)
        spdlog::debug("ShewchukRefiner2D: Failed to insert circumcenter at ({:.2f}, {:.2f}): {}",
                      circumcenter.x(), circumcenter.y(), e.what());
        // Note: Mesh state might be partially modified. The final flood fill will clean up.
        return false; // Skip this triangle
    }
}

} // namespace Meshing
