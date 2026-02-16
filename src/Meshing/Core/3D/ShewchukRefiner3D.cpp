#include "ShewchukRefiner3D.h"
#include "ElementGeometry3D.h"
#include "ElementQuality3D.h"
#include "MeshDebugUtils3D.h"
#include "MeshOperations3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"
#include "Meshing/Data/Base/MeshConnectivity.h"
#include "spdlog/spdlog.h"
#include <algorithm>
#include <cmath>

namespace Meshing
{

ShewchukRefiner3D::ShewchukRefiner3D(MeshingContext3D& context,
                                     const IQualityController3D& qualityController) :
    context_(&context),
    qualityController_(&qualityController)
{
}

ShewchukRefiner3D::~ShewchukRefiner3D() = default;

void ShewchukRefiner3D::refine()
{
    const auto& meshData = context_->getMeshData();

    spdlog::info("ShewchukRefiner3D: Starting mesh refinement");
    spdlog::info("ShewchukRefiner3D: Initial mesh has {} nodes, {} tetrahedra",
                 meshData.getNodeCount(),
                 meshData.getElementCount());
    spdlog::info("ShewchukRefiner3D: Constraints: {} subsegments, {} subfacets",
                 meshData.getConstrainedSubsegmentCount(),
                 meshData.getConstrainedSubfacetCount());

    size_t iterationCount = 0;
    const size_t maxIterations = 10000;
    double qualityBound = qualityController_->getTargetElementQuality();

    exportAndVerifyMesh3D(context_->getMeshData(), MeshingPhase3D::Refined,
                          "ShewchukRefiner3D", exportCounter_, qualityBound);

    while (iterationCount < maxIterations)
    {
        // Check mesh quality
        MeshConnectivity connectivity(context_->getMeshData());
        if (qualityController_->isMeshAcceptable(context_->getMeshData(), connectivity))
        {
            spdlog::info("ShewchukRefiner3D: Mesh quality goals achieved after {} iterations",
                         iterationCount);
            break;
        }

        // Perform one refinement step
        bool refined = refineStep();
        if (!refined)
        {
            spdlog::info("ShewchukRefiner3D: No further refinement possible after {} iterations",
                         iterationCount);
            break;
        }

        // Log progress every 100 iterations
        if (iterationCount % 100 == 0 && iterationCount > 0)
        {
            spdlog::debug("ShewchukRefiner3D: Iteration {}: {} tetrahedra",
                          iterationCount, context_->getMeshData().getElementCount());
        }

        ++iterationCount;
        exportAndVerifyMesh3D(context_->getMeshData(), MeshingPhase3D::Refined,
                              "ShewchukRefiner3D", exportCounter_, qualityBound);
    }

    if (iterationCount >= maxIterations)
    {
        spdlog::warn("ShewchukRefiner3D: Reached maximum iteration limit ({})", maxIterations);
    }

    spdlog::info("ShewchukRefiner3D: Refinement complete. Final mesh: {} nodes, {} tetrahedra",
                 context_->getMeshData().getNodeCount(),
                 context_->getMeshData().getElementCount());
}

bool ShewchukRefiner3D::refineStep()
{
    // Step 1: Handle skinny tetrahedra (Priority 3 in full algorithm)
    // TODO: Add Priority 1 (subsegments) and Priority 2 (subfacets) handling

    ElementQuality3D quality(context_->getMeshData());
    double ratioBound = qualityController_->getTargetElementQuality();
    auto skinnyTets = quality.getSkinnyTetrahedraSortedByQuality(ratioBound);

    for (const auto& [tetId, ratio] : skinnyTets)
    {
        // Skip tetrahedra we've already determined can't be refined
        if (unrefinableTetrahedra_.contains(tetId))
        {
            continue;
        }

        // Check if this tetrahedron still exists
        const auto* element = context_->getMeshData().getElement(tetId);
        if (element == nullptr)
        {
            continue;
        }

        if (handleSkinnyTetrahedron(tetId))
        {
            return true;
        }

        // Mark as unrefinable if we couldn't refine it
        unrefinableTetrahedra_.insert(tetId);
    }

    return false;
}

bool ShewchukRefiner3D::handleSkinnyTetrahedron(size_t tetId)
{
    const auto* element = context_->getMeshData().getElement(tetId);
    const auto* tet = dynamic_cast<const TetrahedralElement*>(element);

    if (tet == nullptr)
    {
        return false;
    }

    // Check if tetrahedron is too small to refine
    if (qualityController_->isTetrahedronTooSmall(*tet))
    {
        spdlog::debug("ShewchukRefiner3D: Tetrahedron {} too small to refine", tetId);
        return false;
    }

    // Compute circumcenter
    ElementGeometry3D geometry(context_->getMeshData());
    auto circumsphereOpt = geometry.computeCircumscribingSphere(*tet);

    if (!circumsphereOpt.has_value())
    {
        spdlog::debug("ShewchukRefiner3D: Cannot compute circumsphere for tetrahedron {}", tetId);
        return false;
    }

    Point3D circumcenter = circumsphereOpt->center;

    // TODO: Check if circumcenter would encroach any subsegments or subfacets
    // For Step 1, we skip this check and just insert the circumcenter

    // Insert the circumcenter using Bowyer-Watson
    try
    {
        context_->getOperations().insertVertexBowyerWatson(circumcenter);
        spdlog::debug("ShewchukRefiner3D: Inserted circumcenter at ({:.4f}, {:.4f}, {:.4f})",
                      circumcenter.x(), circumcenter.y(), circumcenter.z());

        // Clear stale IDs: mesh topology changed, old tet IDs are invalidated
        unrefinableTetrahedra_.clear();

        return true;
    }
    catch (const std::exception& e)
    {
        spdlog::debug("ShewchukRefiner3D: Failed to insert circumcenter: {}", e.what());
        return false;
    }
}

} // namespace Meshing
