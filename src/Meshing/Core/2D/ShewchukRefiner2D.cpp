#include "ShewchukRefiner2D.h"
#include "Common/DebugFlags.h"
#include "Common/Exceptions/MeshException.h"
#include "ElementGeometry2D.h"
#include "ElementQuality2D.h"
#include "Export/VtkExporter.h"
#include "Geometry/2D/Base/GeometryOperations2D.h"
#include "Geometry/2D/Base/IEdge2D.h"
#include "Geometry/2D/Base/IFace2D.h"
#include "MeshOperations2D.h"
#include "Meshing/Core/2D/MeshVerifier.h"
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
                                     const IQualityController2D& qualityController,
                                     const std::vector<ConstrainedSegment2D>& constrainedSegments) :
    context_(&context),
    qualityController_(&qualityController),
    constrainedSegments_(constrainedSegments),
    domainFace_(context.buildDomainFace())
{
}

ShewchukRefiner2D::~ShewchukRefiner2D() = default;

void ShewchukRefiner2D::refine()
{
    spdlog::info("ShewchukRefiner2D: Starting mesh refinement");

    size_t iterationCount = 0;
    size_t consecutiveNoProgress = 0;
    const size_t maxIterations = 10000;         // Safety limit to prevent infinite loops
    const size_t maxConsecutiveNoProgress = 10; // Exit if no progress for this many iterations

    while (true)
    {
        // Check mesh quality
        MeshData3D meshData3D(context_->getMeshData());
        MeshConnectivity connectivity(meshData3D);
        /*
                {
                    Export::VtkExporter exporter;
                    exporter.exportMesh(context_->getMeshData(), "ShewchukRefiner2D_Iteration" + std::to_string(iterationCount) + ".vtu");
                } */

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
                auto interiorTriangles = context_->getOperations().getQueries().classifyTrianglesInteriorExterior(constrainedSegments_);
                auto removedIds = context_->getOperations().removeExteriorTriangles(interiorTriangles);

                // Clean up unrefinableTriangles_ - remove IDs of triangles that were removed
                for (size_t id : removedIds)
                {
                    unrefinableTriangles_.erase(id);
                }
            }
        }

        ++iterationCount;
    }

    spdlog::info("ShewchukRefiner2D: Refinement complete after {} iterations", iterationCount);

    // Re-classify triangles after refinement to remove any that ended up in holes
    // Refinement can create triangles that cross constraint boundaries
    spdlog::info("ShewchukRefiner2D: Re-classifying triangles to remove any in holes");
    auto interiorTriangles = context_->getOperations().getQueries().classifyTrianglesInteriorExterior(constrainedSegments_);
    auto removedIds = context_->getOperations().removeExteriorTriangles(interiorTriangles);

    // Clean up unrefinableTriangles_ - remove IDs of triangles that were removed
    for (size_t id : removedIds)
    {
        unrefinableTriangles_.erase(id);
    }
}

bool ShewchukRefiner2D::refineStep()
{
    // Priority 1: Handle encroached segments first
    auto encroachedSegments = context_->getOperations().getQueries().findEncroachedSegments(constrainedSegments_);

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
        // ShewchukRefiner2D::exportAndVerifyMesh(); // Disabled during refinement - flood fill at end will clean up
        const auto* element = context_->getMeshData().getElement(triangleId);
        const auto* triangle = dynamic_cast<const TriangleElement*>(element);

        if (triangle == nullptr)
            continue;

        // Skip triangles we've already determined can't be refined
        if (unrefinableTriangles_.find(triangleId) != unrefinableTriangles_.end())
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

    // Get the nodes to find the parent edge ID
    const Node2D* node1 = context_->getMeshData().getNode(segment.nodeId1);
    const Node2D* node2 = context_->getMeshData().getNode(segment.nodeId2);

    if (node1 == nullptr || node2 == nullptr)
    {
        spdlog::error("ShewchukRefiner2D: Invalid segment nodes ({}, {})", segment.nodeId1, segment.nodeId2);
        return;
    }

    // Get the geometry ID (edge ID) from the nodes
    // Find the common edge ID between the two nodes
    const auto& geometryIds1 = node1->getGeometryIds();
    const auto& geometryIds2 = node2->getGeometryIds();

    std::string edgeId;

    // Find common geometry ID (the edge both nodes belong to)
    for (const auto& id1 : geometryIds1)
    {
        for (const auto& id2 : geometryIds2)
        {
            if (id1 == id2)
            {
                edgeId = id1;
                break;
            }
        }
        if (!edgeId.empty())
            break;
    }

    if (edgeId.empty())
    {
        spdlog::error("ShewchukRefiner2D: Cannot split segment - no common geometry ID found between nodes {} and {}",
                      segment.nodeId1, segment.nodeId2);
        return;
    }

    // Get the parent edge geometry
    const auto* edge = context_->getGeometry().getEdge(edgeId);
    if (edge == nullptr)
    {
        spdlog::error("ShewchukRefiner2D: Cannot find edge geometry for ID '{}'", edgeId);
        return;
    }

    // Split the segment
    auto splitResult = context_->getOperations().splitConstrainedSegment(segment, *edge, constrainedSegments_);

    if (!splitResult.has_value())
    {
        spdlog::error("ShewchukRefiner2D: Failed to split segment ({}, {})", segment.nodeId1, segment.nodeId2);
        return;
    }

    // Update constrained segments list: remove old segment, add two new ones
    auto it = std::find_if(constrainedSegments_.begin(), constrainedSegments_.end(),
                           [&segment](const ConstrainedSegment2D& s)
                           {
                               return (s.nodeId1 == segment.nodeId1 && s.nodeId2 == segment.nodeId2) ||
                                      (s.nodeId1 == segment.nodeId2 && s.nodeId2 == segment.nodeId1);
                           });

    if (it != constrainedSegments_.end())
    {
        constrainedSegments_.erase(it);
    }

    constrainedSegments_.push_back(splitResult->first);
    constrainedSegments_.push_back(splitResult->second);

    spdlog::debug("ShewchukRefiner2D: Successfully split segment into ({}, {}) and ({}, {})",
                  splitResult->first.nodeId1, splitResult->first.nodeId2,
                  splitResult->second.nodeId1, splitResult->second.nodeId2);
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
    if (!domainFace_ || !Geometry2D::GeometryOperations2D::isPointInsideDomain(circumcenter, *domainFace_))
    {
        spdlog::debug("ShewchukRefiner2D: Circumcenter at ({:.2f}, {:.2f}) is outside domain or in hole, skipping",
                      circumcenter.x(), circumcenter.y());
        return false;
    }

    // Check if circumcenter would encroach any segments
    auto encroachedByCircumcenter = context_->getOperations().getQueries().findSegmentsEncroachedByPoint(circumcenter, constrainedSegments_);

    if (!encroachedByCircumcenter.empty())
    {
        spdlog::debug("ShewchukRefiner2D: Circumcenter would encroach {} segments, splitting them first",
                      encroachedByCircumcenter.size());

        // Split the first encroached segment and defer circumcenter insertion
        handleEncroachedSegment(encroachedByCircumcenter[0]);
        return true; // We made progress by splitting a segment
    }

    // Safe to insert circumcenter
    spdlog::debug("ShewchukRefiner2D: Inserting circumcenter at ({:.2f}, {:.2f})",
                  circumcenter.x(), circumcenter.y());

    try
    {
        context_->getOperations().insertVertexBowyerWatson(circumcenter);
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

void ShewchukRefiner2D::exportAndVerifyMesh()
{
    if (CMESH_DEBUG_ENABLED(EXPORT_MESH_EACH_ITERATION))
    {
        Export::VtkExporter exporter;
        exporter.exportMesh(context_->getMeshData(), "ShewchukRefiner2D_" + std::to_string(exportCounter_++) + ".vtu");
    }

    if (CMESH_DEBUG_ENABLED(CHECK_MESH_EACH_ITERATION))
    {
        spdlog::info("ShewchukRefiner2D: Verifying mesh at export step {}", exportCounter_ - 1);
        MeshVerifier verifier(context_->getMeshData());

        auto result = verifier.verify();
        if (!result.isValid)
        {
            for (const auto& error : result.errors)
            {
                spdlog::error(" - {}", error);
            }
            CMESH_THROW_VERIFICATION_FAILED("Mesh verification failed", result.errors);
        }
    }
}

} // namespace Meshing
