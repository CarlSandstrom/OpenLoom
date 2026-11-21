#include "Meshing/Core/2D/ShewchukRefiner2D.h"
#include "Geometry/2D/Base/IEdge2D.h"
#include "Meshing/Core/2D/General/ElementGeometry2D.h"
#include "Meshing/Core/2D/General/ElementQuality2D.h"
#include "Meshing/Core/2D/General/MeshDebugUtilities2D.h"
#include "Meshing/Core/2D/General/MeshOperations2D.h"
#include "Meshing/Core/2D/General/MeshQueries2D.h"
#include "Meshing/Core/2D/ShewchukQualityController2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/Node2D.h"
#include "Meshing/Data/Base/TriangleElement.h"
#include "spdlog/spdlog.h"
#include <numbers>

namespace Meshing
{

namespace
{

constexpr size_t MAX_ITERATIONS = 10000;
constexpr size_t MAX_CONSECUTIVE_NO_PROGRESS = 10;
constexpr double MIN_REFINABLE_AREA = 1e-12;
constexpr double MIN_REFINABLE_EDGE = 1e-8;
constexpr double COINCIDENT_NODE_TOLERANCE = 1e-8;

bool isTooSmallToRefine(const ElementGeometry2D& geometry,
                        const ElementQuality2D& quality,
                        const TriangleElement& triangle,
                        ElementId triangleId);
bool coincidesWithExistingNode(const MeshData2D& meshData, const Point2D& point);

} // namespace

ShewchukRefiner2D::ShewchukRefiner2D(MeshingContext2D& context,
                                     const MeshQualitySettings2D& qualitySettings,
                                     std::string exportPrefix) :
    context_(&context),
    qualityController_(std::make_unique<ShewchukQualityController2D>(
        context.getMeshData(),
        qualitySettings.circumradiusToEdgeRatio,
        qualitySettings.minAngleDegrees * (std::numbers::pi / 180.0),
        qualitySettings.elementLimit)),
    exportPrefix_(std::move(exportPrefix))
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

    exportAndVerifyMesh(context_->getMeshData(), exportPrefix_ + "ShewchukRefiner2D", exportCounter_);
    while (!qualityController_->isMeshAcceptable(context_->getMeshData()))
    {
        if (iterationCount >= MAX_ITERATIONS)
        {
            spdlog::warn("ShewchukRefiner2D: Reached maximum iteration limit");
            break;
        }

        if (!refineStep())
        {
            consecutiveNoProgress++;
            spdlog::debug("ShewchukRefiner2D: No refinement in iteration {} (consecutive: {})",
                          iterationCount, consecutiveNoProgress);

            if (consecutiveNoProgress >= MAX_CONSECUTIVE_NO_PROGRESS)
            {
                spdlog::warn("ShewchukRefiner2D: Unable to make progress after {} attempts. "
                             "Mesh may contain triangles too small to refine or quality goals may be unachievable.",
                             MAX_CONSECUTIVE_NO_PROGRESS);
                break;
            }
        }
        else
        {
            consecutiveNoProgress = 0;

            // A periodic domain has no exterior to remove.
            if (!context_->getPeriodicData() && iterationCount % 10 == 0)
            {
                spdlog::debug("ShewchukRefiner2D: Periodic triangle re-classification at iteration {}", iterationCount);
                context_->getOperations().classifyAndRemoveExteriorTriangles();
            }
        }

        ++iterationCount;
        exportAndVerifyMesh(context_->getMeshData(), exportPrefix_ + "ShewchukRefiner2D", exportCounter_);
    }

    spdlog::info("ShewchukRefiner2D: Refinement complete after {} iterations", iterationCount);

    // Refinement can create triangles that cross constraint boundaries into holes.
    // A periodic domain has no exterior, so its triangulation is kept whole.
    if (!context_->getPeriodicData())
    {
        spdlog::info("ShewchukRefiner2D: Re-classifying triangles to remove any in holes");
        context_->getOperations().classifyAndRemoveExteriorTriangles();
    }
}

bool ShewchukRefiner2D::refineStep()
{
    auto encroachedSegments = context_->getOperations().getQueries().findEncroachedSegments();

    if (!encroachedSegments.empty())
    {
        spdlog::debug("ShewchukRefiner2D: Found {} encroached segments", encroachedSegments.size());
        return handleEncroachedSegment(encroachedSegments[0]);
    }

    ElementQuality2D quality(context_->getMeshData());
    auto worstTriangles = quality.getTrianglesSortedByQuality();

    for (ElementId triangleId : worstTriangles)
    {
        if (unrefinableTriangles_.contains(triangleId)) continue;

        auto triangle = dynamic_cast<const TriangleElement*>(context_->getMeshData().getElement(triangleId));
        if (triangle == nullptr || qualityController_->isTriangleAcceptable(*triangle)) continue;

        spdlog::debug("ShewchukRefiner2D: Attempting to refine triangle {}", triangleId.value());
        if (handlePoorQualityTriangle(*triangle, triangleId)) return true;

        // Retrying a triangle that failed once would loop forever.
        unrefinableTriangles_.insert(triangleId);
    }

    return false;
}

bool ShewchukRefiner2D::handleEncroachedSegment(const CurveSegment& segment)
{
    spdlog::debug("ShewchukRefiner2D: Splitting encroached segment ({}, {})",
                  segment.startNodeId.value(), segment.endNodeId.value());

    auto edge = context_->getGeometry().getEdge(segment.edgeId);
    auto newNodeId = context_->getOperations().splitConstrainedSegment(segment, *edge);

    if (!newNodeId.has_value())
    {
        spdlog::error("ShewchukRefiner2D: Failed to split segment ({}, {})", segment.startNodeId.value(), segment.endNodeId.value());
        return false;
    }

    spdlog::debug("ShewchukRefiner2D: Successfully split segment ({}, {}) at node {}",
                  segment.startNodeId.value(), segment.endNodeId.value(), newNodeId->value());

    if (onBoundarySplit_)
    {
        onBoundarySplit_(segment.startNodeId, segment.endNodeId, newNodeId.value());
    }
    return true;
}

bool ShewchukRefiner2D::handlePoorQualityTriangle(const TriangleElement& triangle, ElementId triangleId)
{
    ElementGeometry2D geometry(context_->getMeshData(), context_->getPeriodicData());
    ElementQuality2D quality(context_->getMeshData());

    if (isTooSmallToRefine(geometry, quality, triangle, triangleId)) return false;

    // Offset-aware coordinates; a no-op for a non-periodic mesh.
    auto optionalCircumcenter = geometry.computeCircumcenter(triangleId);

    if (!optionalCircumcenter.has_value())
    {
        spdlog::debug("ShewchukRefiner2D: Cannot compute circumcenter for triangle {} (likely degenerate), skipping", triangleId.value());
        return false;
    }

    Point2D circumcenter = optionalCircumcenter.value();

    // Per Ruppert: a circumcenter that would encroach a visible subsegment is
    // rejected, and the subsegment is split instead. This comes before the
    // domain test: a circumcenter outside the domain encroaches the boundary
    // subsegment between it and the triangle, and splitting that is how the
    // triangle gets refined.
    auto encroachedByCircumcenter = context_->getOperations().getQueries().findSegmentsEncroachedByPoint(circumcenter);

    if (!encroachedByCircumcenter.empty())
    {
        spdlog::debug("ShewchukRefiner2D: Circumcenter would encroach {} segments, splitting them first",
                      encroachedByCircumcenter.size());

        return handleEncroachedSegment(encroachedByCircumcenter[0]);
    }

    if (!context_->getOperations().getQueries().isPointInsideDomain(circumcenter))
    {
        spdlog::debug("ShewchukRefiner2D: Circumcenter at ({:.2f}, {:.2f}) is outside domain or in hole, skipping",
                      circumcenter.x(), circumcenter.y());
        return false;
    }

    // A circumcenter can land on a circle center where a node already sits,
    // and Bowyer-Watson cannot insert a duplicate point.
    if (coincidesWithExistingNode(context_->getMeshData(), circumcenter)) return false;

    spdlog::debug("ShewchukRefiner2D: Inserting circumcenter at ({:.2f}, {:.2f})",
                  circumcenter.x(), circumcenter.y());

    if (!context_->getOperations().insertVertexBowyerWatson(circumcenter))
    {
        spdlog::debug("ShewchukRefiner2D: Constrained edges block circumcenter at ({:.2f}, {:.2f})",
                      circumcenter.x(), circumcenter.y());
        return false;
    }

    // A triangle that survives the insertion had its circumcenter on the other
    // side of a constrained edge; refining it again would loop forever.
    if (context_->getMeshData().getElement(triangleId) != nullptr)
    {
        spdlog::debug("ShewchukRefiner2D: Triangle {} survived circumcenter insertion, marking unrefinable",
                      triangleId.value());
        unrefinableTriangles_.insert(triangleId);
    }
    return true;
}

namespace
{

// Refining very small triangles runs into numerical trouble.
bool isTooSmallToRefine(const ElementGeometry2D& geometry,
                        const ElementQuality2D& quality,
                        const TriangleElement& triangle,
                        ElementId triangleId)
{
    double area = geometry.computeArea(triangle);
    double shortestEdge = quality.computeShortestEdgeLength(triangle);

    if (area < MIN_REFINABLE_AREA || shortestEdge < MIN_REFINABLE_EDGE)
    {
        spdlog::debug("ShewchukRefiner2D: Triangle {} too small to refine (area={:.2e}, shortest edge={:.2e}), skipping",
                      triangleId.value(), area, shortestEdge);
        return true;
    }
    return false;
}

bool coincidesWithExistingNode(const MeshData2D& meshData, const Point2D& point)
{
    for (auto& [nodeId, node] : meshData.getNodes())
    {
        double distance = (node->getCoordinates() - point).norm();
        if (distance < COINCIDENT_NODE_TOLERANCE)
        {
            spdlog::debug("ShewchukRefiner2D: Circumcenter at ({:.2f}, {:.2f}) coincides with "
                          "existing node {}, skipping",
                          point.x(), point.y(), nodeId.value());
            return true;
        }
    }
    return false;
}

} // namespace

} // namespace Meshing
