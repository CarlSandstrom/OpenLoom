#include "ShewchukRefiner2D.h"
#include "Computer2D.h"
#include "MeshOperations2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/Node2D.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/Base/MeshConnectivity.h"
#include "spdlog/spdlog.h"

namespace Meshing
{

ShewchukRefiner2D::ShewchukRefiner2D(MeshingContext2D& context,
                                     const IQualityController2D& qualityController,
                                     const std::vector<ConstrainedSegment2D>& constrainedSegments) :
    context_(&context),
    qualityController_(&qualityController),
    constrainedSegments_(constrainedSegments)
{
}

void ShewchukRefiner2D::refine()
{
    spdlog::info("ShewchukRefiner2D: Starting mesh refinement");

    size_t iterationCount = 0;
    const size_t maxIterations = 10000; // Safety limit to prevent infinite loops

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
            spdlog::info("ShewchukRefiner2D: No more refinements possible");
            break;
        }

        ++iterationCount;
    }

    spdlog::info("ShewchukRefiner2D: Refinement complete after {} iterations", iterationCount);
}

bool ShewchukRefiner2D::refineStep()
{
    // Priority 1: Handle encroached segments first
    auto encroachedSegments = findEncroachedSegments();

    if (!encroachedSegments.empty())
    {
        spdlog::debug("ShewchukRefiner2D: Found {} encroached segments", encroachedSegments.size());
        handleEncroachedSegment(encroachedSegments[0]);
        return true;
    }

    // Priority 2: Refine worst quality triangle
    Computer2D computer(context_->getMeshData());
    auto worstTriangles = computer.getTrianglesSortedByQuality();

    if (!worstTriangles.empty())
    {
        size_t worstTriangleId = worstTriangles[0];

        // Check if this triangle is acceptable
        const auto* element = context_->getMeshData().getElement(worstTriangleId);
        const auto* triangle = dynamic_cast<const TriangleElement*>(element);

        if (triangle != nullptr && !qualityController_->isTriangleAcceptable(*triangle))
        {
            spdlog::debug("ShewchukRefiner2D: Refining triangle {}", worstTriangleId);
            handlePoorQualityTriangle(worstTriangleId);
            return true;
        }
    }

    return false;
}

std::vector<ConstrainedSegment2D> ShewchukRefiner2D::findEncroachedSegments() const
{
    std::vector<ConstrainedSegment2D> encroached;

    Computer2D computer(context_->getMeshData());

    // Check each constrained segment against all nodes
    for (const auto& segment : constrainedSegments_)
    {
        // Check if any node encroaches this segment
        for (const auto& [nodeId, node] : context_->getMeshData().getNodes())
        {
            // Skip the segment endpoints
            if (nodeId == segment.nodeId1 || nodeId == segment.nodeId2)
                continue;

            Point2D point = node->getCoordinates();

            if (computer.isSegmentEncroached(segment, point))
            {
                encroached.push_back(segment);
                break; // This segment is encroached, move to next segment
            }
        }
    }

    return encroached;
}

void ShewchukRefiner2D::handleEncroachedSegment(const ConstrainedSegment2D& segment)
{
    spdlog::debug("ShewchukRefiner2D: Splitting encroached segment ({}, {})",
                  segment.nodeId1, segment.nodeId2);

    // TODO: Need to find the parent edge geometry for this segment
    // For now, this is a placeholder - we'll need to track edge geometry
    // in the constrained segments or look it up from the mesh context

    // The segment splitting will be implemented once we have access to the parent edge
    spdlog::error("ShewchukRefiner2D: Segment splitting not yet implemented - need parent edge");
}

void ShewchukRefiner2D::handlePoorQualityTriangle(size_t triangleId)
{
    const auto* element = context_->getMeshData().getElement(triangleId);
    const auto* triangle = dynamic_cast<const TriangleElement*>(element);

    if (triangle == nullptr)
    {
        spdlog::error("ShewchukRefiner2D: Triangle {} not found", triangleId);
        return;
    }

    // Compute circumcenter
    Computer2D computer(context_->getMeshData());
    auto circumcenterOpt = computer.computeCircumcenter(*triangle);

    if (!circumcenterOpt.has_value())
    {
        spdlog::error("ShewchukRefiner2D: Failed to compute circumcenter for triangle {}", triangleId);
        return;
    }

    Point2D circumcenter = circumcenterOpt.value();

    // Check if circumcenter would encroach any segments
    auto encroachedByCircumcenter = findSegmentsEncroachedByPoint(circumcenter);

    if (!encroachedByCircumcenter.empty())
    {
        spdlog::debug("ShewchukRefiner2D: Circumcenter would encroach {} segments, splitting them first",
                      encroachedByCircumcenter.size());

        // Split the first encroached segment and defer circumcenter insertion
        handleEncroachedSegment(encroachedByCircumcenter[0]);
        return;
    }

    // Safe to insert circumcenter
    spdlog::debug("ShewchukRefiner2D: Inserting circumcenter at ({:.2f}, {:.2f})",
                  circumcenter.x(), circumcenter.y());

    context_->getOperations().insertVertexBowyerWatson(circumcenter);
}

std::vector<ConstrainedSegment2D> ShewchukRefiner2D::findSegmentsEncroachedByPoint(const Point2D& point) const
{
    std::vector<ConstrainedSegment2D> encroached;

    Computer2D computer(context_->getMeshData());

    for (const auto& segment : constrainedSegments_)
    {
        if (computer.isSegmentEncroached(segment, point))
        {
            encroached.push_back(segment);
        }
    }

    return encroached;
}

} // namespace Meshing
